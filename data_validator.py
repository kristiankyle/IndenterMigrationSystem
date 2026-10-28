"""
data_validator.py
Stage 1B: Data integrity validator for RawFileData.
Checks row/column counts, missing cells, non-numeric entries, and computes
column-level min/max/mean statistics without modifying or silently deleting rows.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np

from sample_model import RawFileData


@dataclass
class ColumnValidationStats:
    """Summary integrity and range statistics for a single column."""
    col_index: int
    col_name: str
    total_cells: int
    numeric_count: int
    missing_count: int
    non_numeric_count: int
    min_val: Optional[float] = None
    max_val: Optional[float] = None
    mean_val: Optional[float] = None
    initial_val: Optional[float] = None

    @property
    def numeric_ratio(self) -> float:
        return (self.numeric_count / self.total_cells) if self.total_cells > 0 else 0.0


@dataclass
class ValidationReport:
    """Complete validation result for a loaded RawFileData table."""
    file_name: str
    is_valid: bool
    total_rows: int
    total_cols: int
    column_stats: List[ColumnValidationStats] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)


def validate_raw_data(raw_data: RawFileData) -> ValidationReport:
    """
    Inspects every row and column of RawFileData and produces a ValidationReport.
    Never modifies the original file or silently drops rows.
    """
    warnings: List[str] = []
    errors: List[str] = []
    col_stats_list: List[ColumnValidationStats] = []

    if raw_data.total_rows < 2:
        errors.append(
            f"File contains only {raw_data.total_rows} data row(s); at least 2 rows are required."
        )

    if raw_data.total_cols < 2:
        errors.append(
            f"File contains only {raw_data.total_cols} column(s); at least 2 columns (Position & Force/Voltage) are required."
        )

    for col_idx in range(raw_data.total_cols):
        col_name = raw_data.column_names[col_idx]
        numeric_vals: List[float] = []
        missing_cnt = 0
        non_numeric_cnt = 0

        for row in raw_data.rows:
            cell = row[col_idx].strip() if col_idx < len(row) else ""
            if cell == "" or cell.lower() in ("nan", "null", "none"):
                missing_cnt += 1
            else:
                try:
                    val = float(cell)
                    if np.isnan(val) or np.isinf(val):
                        missing_cnt += 1
                    else:
                        numeric_vals.append(val)
                except ValueError:
                    non_numeric_cnt += 1

        if numeric_vals:
            arr = np.asarray(numeric_vals, dtype=float)
            c_min = float(np.min(arr))
            c_max = float(np.max(arr))
            c_mean = float(np.mean(arr))
            c_init = float(arr[0])
        else:
            c_min = c_max = c_mean = c_init = None

        stats = ColumnValidationStats(
            col_index=col_idx,
            col_name=col_name,
            total_cells=raw_data.total_rows,
            numeric_count=len(numeric_vals),
            missing_count=missing_cnt,
            non_numeric_count=non_numeric_cnt,
            min_val=c_min,
            max_val=c_max,
            mean_val=c_mean,
            initial_val=c_init,
        )
        col_stats_list.append(stats)

        if missing_cnt > 0:
            warnings.append(
                f"Column '{col_name}' has {missing_cnt} missing/blank cell(s) (preserved as NaN)."
            )
        if non_numeric_cnt > 0:
            warnings.append(
                f"Column '{col_name}' has {non_numeric_cnt} non-numeric text cell(s)."
            )
        if stats.numeric_ratio < 0.5:
            errors.append(
                f"Column '{col_name}' contains less than 50% valid numbers ({stats.numeric_count}/{stats.total_cells})."
            )

    is_valid = len(errors) == 0
    return ValidationReport(
        file_name=raw_data.file_name,
        is_valid=is_valid,
        total_rows=raw_data.total_rows,
        total_cols=raw_data.total_cols,
        column_stats=col_stats_list,
        warnings=warnings,
        errors=errors,
    )


def format_validation_report(report: ValidationReport) -> str:
    """Formats the ValidationReport as a readable summary block."""
    status_str = "PASSED (VALID)" if report.is_valid else "FAILED (ERRORS FOUND)"
    lines = [
        f"Validation Status: {status_str}",
        f"Checked Table:     {report.total_rows} rows x {report.total_cols} columns",
        "-" * 72,
        "Column Integrity & Range Summary:",
    ]

    for st in report.column_stats:
        if st.min_val is not None:
            range_str = (
                f"Start={st.initial_val:.3f} | Min={st.min_val:.3f} | "
                f"Max={st.max_val:.3f} | Mean={st.mean_val:.3f}"
            )
        else:
            range_str = "No numeric values"

        lines.append(
            f"  [{st.col_index}] {st.col_name:<30} "
            f"Valid: {st.numeric_count}/{st.total_cells} "
            f"(Missing: {st.missing_count}, Text: {st.non_numeric_count})\n"
            f"      -> {range_str}"
        )

    if report.warnings:
        lines.append("-" * 72)
        lines.append("Warnings:")
        for w in report.warnings:
            lines.append(f"  ! {w}")

    if report.errors:
        lines.append("-" * 72)
        lines.append("Errors:")
        for e in report.errors:
            lines.append(f"  [X] {e}")

    return "\n".join(lines)