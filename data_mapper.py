"""
data_mapper.py
Stage 1B: Automatic and customizable column mapper & unit converter.
Transforms RawFileData into a standardized SampleData instance with:
  - time (s)
  - position (mm)
  - force (g)
  - voltage (V)
Supports both 2-column legacy LabVIEW files and multi-column CSV/Excel exports.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional
import numpy as np

from sample_model import RawFileData, SampleData


DEFAULT_STEPS_PER_MM: float = 1600.0
DEFAULT_GRAMS_PER_VOLT: float = 505.97
DEFAULT_LABVIEW_DT_S: float = 0.10


@dataclass
class ColumnMappingConfig:
    """Specifies how raw columns map to standardized SampleData fields and units."""
    time_col: Optional[int] = None
    position_col: Optional[int] = None
    force_col: Optional[int] = None
    voltage_col: Optional[int] = None

    # Input units of the source columns
    position_unit: str = "steps"   # "steps", "mm", or "um"
    force_unit: str = "V_raw"      # "g", "N", or "V_raw" (convert from voltage)
    zero_position_baseline: bool = True
    tare_voltage_baseline: bool = True
    steps_per_mm: float = DEFAULT_STEPS_PER_MM
    grams_per_volt: float = DEFAULT_GRAMS_PER_VOLT
    assumed_dt_s: float = DEFAULT_LABVIEW_DT_S


def _extract_float_column(raw_data: RawFileData, col_idx: int) -> np.ndarray:
    """Extracts a column as a 1D float NumPy array, preserving missing cells as np.nan."""
    values: List[float] = []
    for row in raw_data.rows:
        cell = row[col_idx].strip() if col_idx < len(row) else ""
        if cell == "":
            values.append(np.nan)
        else:
            try:
                values.append(float(cell))
            except ValueError:
                values.append(np.nan)
    return np.asarray(values, dtype=float)


def auto_detect_mapping(raw_data: RawFileData) -> ColumnMappingConfig:
    """
    Automatically determines column roles and units based on column names
    or legacy 2-column LabVIEW file structure.
    """
    # Case 1: Headerless 2-column LabVIEW file (Column_0 = Steps, Column_1 = Voltage)
    if not raw_data.has_header_row and raw_data.total_cols == 2:
        return ColumnMappingConfig(
            time_col=None,
            position_col=0,
            force_col=None,
            voltage_col=1,
            position_unit="steps",
            force_unit="V_raw",
            zero_position_baseline=True,
            tare_voltage_baseline=True,
        )

    # Case 2: Named columns (Python S4 CSV export, custom CSV, or Excel)
    cfg = ColumnMappingConfig(
        zero_position_baseline=False,
        tare_voltage_baseline=False,
    )

    for idx, name in enumerate(raw_data.column_names):
        low = name.lower()
        if cfg.time_col is None and any(k in low for k in ("time", "elapsed", "sec")):
            cfg.time_col = idx
        elif any(k in low for k in ("disp", "mm")):
            cfg.position_col = idx
            cfg.position_unit = "mm"
        elif cfg.position_col is None and any(k in low for k in ("pos", "step", "stroke")):
            cfg.position_col = idx
            cfg.position_unit = "steps" if "step" in low else "mm"
        elif any(k in low for k in ("force_g", "force (g)", "load_g", "gram")):
            cfg.force_col = idx
            cfg.force_unit = "g"
        elif cfg.force_col is None and any(k in low for k in ("force", "load")):
            cfg.force_col = idx
            cfg.force_unit = "N" if ("_n" in low or "(n)" in low) else "g"
        elif cfg.voltage_col is None and any(k in low for k in ("volt", "daq", "_v")):
            cfg.voltage_col = idx

    # Fallback if 2 columns exist with unrecognized headers
    if cfg.position_col is None and raw_data.total_cols >= 2:
        cfg.position_col = 0
        cfg.position_unit = "steps"
        cfg.zero_position_baseline = True
    if cfg.force_col is None and cfg.voltage_col is None and raw_data.total_cols >= 2:
        cfg.voltage_col = 1
        cfg.force_unit = "V_raw"
        cfg.tare_voltage_baseline = True

    return cfg


def map_raw_to_sample_data(
    raw_data: RawFileData,
    mapping: Optional[ColumnMappingConfig] = None,
) -> SampleData:
    """
    Applies ColumnMappingConfig to RawFileData and returns a standardized SampleData
    object with time (s), position (mm), force (g), and voltage (V).
    """
    if mapping is None:
        mapping = auto_detect_mapping(raw_data)

    transformations: List[str] = []
    n_rows = raw_data.total_rows

    # 1. Position -> standardized to millimeters (mm)
    pos_mm: Optional[np.ndarray] = None
    if mapping.position_col is not None:
        raw_pos = _extract_float_column(raw_data, mapping.position_col)
        if mapping.zero_position_baseline and len(raw_pos) > 0 and not np.isnan(raw_pos[0]):
            baseline_pos = float(raw_pos[0])
            raw_pos = raw_pos - baseline_pos
            transformations.append(
                f"Zeroed initial position baseline ({baseline_pos:.1f} {mapping.position_unit} -> 0.0)"
            )

        if mapping.position_unit == "steps":
            pos_mm = raw_pos / mapping.steps_per_mm
            transformations.append(
                f"Converted position from steps to mm (/ {mapping.steps_per_mm:.1f} steps/mm)"
            )
        elif mapping.position_unit == "um":
            pos_mm = raw_pos / 1000.0
            transformations.append("Converted position from um to mm (/ 1000)")
        else:
            pos_mm = raw_pos

    # 2. Voltage (V) & Force -> standardized to grams (g)
    voltage_v: Optional[np.ndarray] = None
    force_g: Optional[np.ndarray] = None

    if mapping.voltage_col is not None:
        voltage_v = _extract_float_column(raw_data, mapping.voltage_col)

    if mapping.force_col is not None:
        raw_force = _extract_float_column(raw_data, mapping.force_col)
        if mapping.force_unit == "N":
            force_g = (raw_force / 9.80665) * 1000.0
            transformations.append("Converted force from N to g")
        else:
            force_g = raw_force
    elif voltage_v is not None:
        v_work = voltage_v.copy()
        if mapping.tare_voltage_baseline and len(v_work) > 0 and not np.isnan(v_work[0]):
            v_tare = float(np.nanmean(v_work[:5]))
            v_work = v_work - v_tare
            transformations.append(
                f"Tared initial voltage baseline (V_tare = {v_tare:.5f} V)"
            )
        # Note: In legacy LabVIEW files, if indentation causes negative or positive voltage,
        # orient force so loading peak is positive if dominant excursion is negative
        if np.nanmax(np.abs(v_work)) > 0:
            max_pos = float(np.nanmax(v_work))
            max_neg = float(np.abs(np.nanmin(v_work)))
            if max_neg > max_pos * 1.5:
                v_work = -v_work
                transformations.append("Inverted polarity so compressive indentation force is positive (+g)")

        force_g = v_work * mapping.grams_per_volt
        transformations.append(
            f"Converted voltage to force in grams (* {mapping.grams_per_volt:.2f} g/V)"
        )

    # 3. Time -> standardized to seconds (s)
    if mapping.time_col is not None:
        time_s = _extract_float_column(raw_data, mapping.time_col)
    else:
        time_s = np.arange(n_rows, dtype=float) * mapping.assumed_dt_s
        transformations.append(
            f"Synthesized time axis for headerless LabVIEW file (dt = {mapping.assumed_dt_s:.2f} s/sample)"
        )

    metadata = {
        "source_path": raw_data.file_path,
        "has_header_row": raw_data.has_header_row,
        "original_columns": raw_data.column_names,
        "total_rows": n_rows,
        "steps_per_mm": mapping.steps_per_mm,
        "grams_per_volt": mapping.grams_per_volt,
    }

    return SampleData(
        sample_id=raw_data.file_name,
        file_name=raw_data.file_name,
        file_type=raw_data.file_type,
        time=time_s,
        position=pos_mm,
        force=force_g,
        voltage=voltage_v,
        metadata=metadata,
        transformations_applied=transformations,
        raw_file_data=raw_data,
    )


def format_mapped_sample_summary(sample: SampleData) -> str:
    """Formats a summary of the standardized SampleData arrays and transformations."""
    lines = [
        "Standardized SampleData Summary:",
        f"  Sample ID:        {sample.sample_id} ({sample.num_samples} samples)",
    ]

    if sample.time is not None and len(sample.time) > 0:
        lines.append(
            f"  Time (s):         0.00 s -> {float(np.nanmax(sample.time)):.2f} s"
        )
    if sample.position is not None and len(sample.position) > 0:
        lines.append(
            f"  Position (mm):    Min = {float(np.nanmin(sample.position)):.3f} mm | "
            f"Max = {float(np.nanmax(sample.position)):.3f} mm"
        )
    if sample.force is not None and len(sample.force) > 0:
        lines.append(
            f"  Force (g):        Min = {float(np.nanmin(sample.force)):.2f} g  | "
            f"Max = {float(np.nanmax(sample.force)):.2f} g"
        )
    if sample.voltage is not None and len(sample.voltage) > 0:
        lines.append(
            f"  Raw Voltage (V):  Min = {float(np.nanmin(sample.voltage)):.4f} V | "
            f"Max = {float(np.nanmax(sample.voltage)):.4f} V"
        )

    if sample.transformations_applied:
        lines.append("  Transformations Applied:")
        for tr in sample.transformations_applied:
            lines.append(f"    * {tr}")

    return "\n".join(lines)