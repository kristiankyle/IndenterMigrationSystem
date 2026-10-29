"""
s4_graph_and_export.py
Replaces LabVIEW 'S4_Youngs_Graph.vi'.
When the operator clicks the Stage S4 Export button in gui.py, this module:
  1. Opens a 'Save As' dialog so the user can freely type or rename the filename.
  2. Automatically generates all 3 formats from that filename:
     - <SampleName>      : Extensionless 'File' (2-col headerless %.3f\t%.3f for MATLAB)
     - <SampleName>.mat  : Native MATLAB Level 5 binary workspace (.mat) with metadata
     - <SampleName>.csv  : Calibrated multi-column CSV for Excel & Python Sample Reader
"""

import csv
import os
import struct
from datetime import datetime
from typing import Any, Dict, Tuple
import numpy as np
import matplotlib.pyplot as plt
from tkinter import filedialog, messagebox

import config
from state import GlobalSystemState


KNOWN_EXTENSIONS = (".txt", ".csv", ".mat", ".dat", ".tsv", ".xlsx")


def _strip_known_extension(filepath: str) -> str:
    """
    Removes any trailing .txt, .csv, or .mat extension so the base path
    matches the exact sample name typed by the user (e.g., RUBIN16036AGM4).
    """
    base, ext = os.path.splitext(filepath)
    if ext.lower() in KNOWN_EXTENSIONS:
        return base
    return filepath


def _write_matlab_v5_mat(filepath: str, variables: Dict[str, Any]) -> None:
    """
    Writes a standard MATLAB Level 5 (.mat) binary file using pure Python + NumPy.
    Works on Windows 7 & macOS without requiring scipy to be installed.
    """
    try:
        from scipy.io import savemat
        savemat(filepath, variables, do_compression=False)
        return
    except ImportError:
        pass

    def _pad8(data_bytes: bytes) -> bytes:
        rem = len(data_bytes) % 8
        return data_bytes + (b"\x00" * (8 - rem) if rem != 0 else b"")

    def _pack_matrix(name: str, value: Any) -> bytes:
        arr = np.asarray(value, dtype="<f8")
        if arr.ndim == 0:
            arr = arr.reshape((1, 1))
        elif arr.ndim == 1:
            arr = arr.reshape((-1, 1))

        rows, cols = arr.shape
        fortran_bytes = np.asfortranarray(arr, dtype="<f8").tobytes()

        # 1. Array Flags (miUINT32 = 6, length = 8, mxDOUBLE_CLASS = 6)
        flags_block = struct.pack("<IIII", 6, 8, 6, 0)
        # 2. Dimensions Array (miINT32 = 5, length = 8)
        dims_block = struct.pack("<IIII", 5, 8, int(rows), int(cols))
        # 3. Array Name (miINT8 = 1)
        name_ascii = name.encode("ascii", errors="ignore")[:63]
        name_block = struct.pack("<II", 1, len(name_ascii)) + _pad8(name_ascii)
        # 4. Real Part Data (miDOUBLE = 9)
        data_block = struct.pack("<II", 9, len(fortran_bytes)) + _pad8(fortran_bytes)

        body = flags_block + dims_block + name_block + data_block
        return struct.pack("<II", 14, len(body)) + body

    header_text = (
        f"MATLAB 5.0 MAT-file, Platform: Python Indenter S4, "
        f"Created on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
    ).encode("ascii")
    header_116 = header_text[:116].ljust(116, b" ")
    subsys_offset = b"\x00" * 8
    version_endian = struct.pack("<H", 0x0100) + b"IM"
    header_128 = header_116 + subsys_offset + version_endian

    with open(filepath, "wb") as f:
        f.write(header_128)
        for var_name, var_val in variables.items():
            f.write(_pack_matrix(var_name, var_val))


def calculate_stiffness_and_modulus(
    state: GlobalSystemState,
    indenter_radius_mm: float = 1.5,
    poisson_ratio: float = 0.45,
) -> Dict[str, float]:
    """
    Computes contact stiffness slope (dF/dh) from the displacement-force curve
    and effective flat-ended cylindrical indenter Young's Modulus (E):
      E = (1 - nu^2) / (2 * R) * (dF / dh)
    """
    disp_mm = np.array(state.history_displacement_mm, dtype=float)
    force_g = np.array(state.history_force_g, dtype=float)
    force_n = (force_g / 1000.0) * 9.80665

    if len(disp_mm) < 2 or np.ptp(disp_mm) < 1e-6:
        return {
            "stiffness_g_per_mm": 0.0,
            "stiffness_n_per_mm": 0.0,
            "youngs_modulus_kpa": 0.0,
        }

    slope_g_mm, _ = np.polyfit(disp_mm, force_g, 1)
    slope_n_mm, _ = np.polyfit(disp_mm, force_n, 1)

    e_mpa = ((1.0 - poisson_ratio ** 2) / (2.0 * indenter_radius_mm)) * slope_n_mm
    e_kpa = float(e_mpa * 1000.0)

    return {
        "stiffness_g_per_mm": float(slope_g_mm),
        "stiffness_n_per_mm": float(slope_n_mm),
        "youngs_modulus_kpa": e_kpa,
    }


def export_s4_spreadsheets(
    state: GlobalSystemState, base_filename: str = "RUBIN_SAMPLE"
) -> Tuple[str, str]:
    """
    Called directly by the Stage S4 Export button in gui.py.
    1. Prompts the user with a 'Save As' window to enter or edit the sample filename.
    2. Generates all 3 files simultaneously:
       - <SampleName>     : Extensionless 2-col tab-separated 'File' (1000.000\t0.079) for MATLAB
       - <SampleName>.mat : Native MATLAB v5 workspace file (.mat) with full data & metadata
       - <SampleName>.csv : Full Calibrated Engineering CSV (.csv)
    """
    if not state.history_position_steps:
        raise ValueError("No recorded S3 data available to export in S4.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_sample_name = f"{base_filename}_{timestamp}"

    chosen_path = filedialog.asksaveasfilename(
        title="Enter Sample Filename (Saves MATLAB 'File', .mat, and .csv)",
        initialfile=default_sample_name,
        filetypes=[
            ("All 3 Formats: MATLAB 'File' (No Ext) + .mat + .csv", "*"),
            ("MATLAB Workspace (.mat)", "*.mat"),
            ("Calibrated CSV (.csv)", "*.csv"),
        ],
    )

    if not chosen_path:
        raise RuntimeError("Export cancelled by user.")

    base_path = _strip_known_extension(chosen_path.strip())
    parent_dir = os.path.dirname(base_path)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)

    raw_lv_file = base_path                 # 1. Extensionless "File" for MATLAB
    mat_file = base_path + ".mat"           # 2. Native MATLAB .mat file
    calibrated_csv_file = base_path + ".csv"  # 3. Calibrated .csv file

    n = len(state.history_position_steps)
    time_arr = np.asarray(state.history_timestamp_s, dtype=float)
    steps_arr = np.asarray(state.history_position_steps, dtype=float)
    disp_mm_arr = np.asarray(state.history_displacement_mm, dtype=float)
    raw_v_arr = np.asarray(state.history_voltage_v, dtype=float)
    force_g_arr = np.asarray(state.history_force_g, dtype=float)
    force_n_arr = (force_g_arr / 1000.0) * 9.80665

    v_tare = float(getattr(state, "v_tare", 0.0))
    g_per_v = float(getattr(state, "grams_per_volt", config.CALIBRATED_GRAMS_PER_VOLT))
    tared_v_arr = raw_v_arr - v_tare
    preload_g = v_tare * g_per_v

    metrics = calculate_stiffness_and_modulus(state)

    # -------------------------------------------------------------------------
    # 1. Extensionless MATLAB / LabVIEW "File" (2 headerless cols: %.3f\t%.3f)
    #    Exact match to RUBIN16036AGM2: 1000.000\t0.079
    # -------------------------------------------------------------------------
    with open(raw_lv_file, mode="w", encoding="ascii", newline="\r\n") as f_lv:
        for pos_steps, volt_v in zip(steps_arr, raw_v_arr):
            f_lv.write(f"{float(pos_steps):.3f}\t{float(volt_v):.3f}\n")

    # -------------------------------------------------------------------------
    # 2. Native MATLAB Level 5 Workspace File (.mat)
    # -------------------------------------------------------------------------
    raw_labview_matrix = np.column_stack((steps_arr, raw_v_arr))
    calibrated_matrix = np.column_stack(
        (time_arr, steps_arr, disp_mm_arr, raw_v_arr, tared_v_arr, force_g_arr, force_n_arr)
    )
    mat_payload: Dict[str, Any] = {
        "raw_labview_data": raw_labview_matrix,
        "calibrated_data": calibrated_matrix,
        "Time_s": time_arr,
        "Position_steps": steps_arr,
        "Displacement_mm": disp_mm_arr,
        "Voltage_Raw_V": raw_v_arr,
        "Voltage_Tared_V": tared_v_arr,
        "Force_g": force_g_arr,
        "Force_N": force_n_arr,
        "Stiffness_g_per_mm": metrics["stiffness_g_per_mm"],
        "Stiffness_N_per_mm": metrics["stiffness_n_per_mm"],
        "Youngs_Modulus_kPa": metrics["youngs_modulus_kpa"],
        "Conversion_Factor_g_per_V": g_per_v,
        "Tare_Voltage_V": v_tare,
        "Preload_Force_g": preload_g,
        "Steps_per_mm": float(config.STEPS_PER_MM),
        "Advance_mm": float(state.advance_distance_mm),
        "Rise_Time_s": float(state.rise_time_s),
        "Dwell_Time_s": float(state.dwell_time_s),
        "Repeat_Cycles": float(state.repeat_cycles),
    }
    _write_matlab_v5_mat(mat_file, mat_payload)

    # -------------------------------------------------------------------------
    # 3. Full Calibrated Engineering CSV (.csv)
    # -------------------------------------------------------------------------
    with open(calibrated_csv_file, mode="w", encoding="utf-8", newline="") as f_csv:
        writer = csv.writer(f_csv)
        writer.writerow([
            "Time_s",
            "Position_steps",
            "Displacement_mm",
            "Voltage_V",
            "Force_g",
            "Force_N",
        ])
        for i in range(n):
            writer.writerow([
                f"{time_arr[i]:.4f}",
                f"{int(round(steps_arr[i]))}",
                f"{disp_mm_arr[i]:.5f}",
                f"{raw_v_arr[i]:.6f}",
                f"{force_g_arr[i]:.3f}",
                f"{force_n_arr[i]:.5f}",
            ])

    messagebox.showinfo(
        "Stage S4 Export Complete (3 Formats Saved)",
        f"Saved all 3 formats using your chosen name:\n\n"
        f"1. MATLAB 'File' (No Extension, 2-Col Raw):\n{raw_lv_file}\n\n"
        f"2. Native MATLAB Workspace (.mat):\n{mat_file}\n\n"
        f"3. Calibrated Spreadsheet (.csv):\n{calibrated_csv_file}",
    )

    return raw_lv_file, calibrated_csv_file


def plot_s4_graphs(state: GlobalSystemState) -> None:
    """
    Reproduces both graphs from S4_Youngs_Graph.vi Front Panel:
      - Chart of Force & Displacement vs. Time
      - X-Y Plot (Displacement vs. Force)
    """
    fig, (ax_disp, ax_force, ax_xy) = plt.subplots(3, 1, figsize=(8, 9))
    fig.suptitle("S4_Youngs_Graph: Force & Displacement Analysis", fontsize=14, fontweight="bold")

    ax_disp.plot(
        state.history_timestamp_s,
        state.history_position_steps,
        color="white",
        linewidth=1.8,
    )
    ax_disp.set_facecolor("black")
    ax_disp.set_ylabel("Displacement (steps)")
    ax_disp.set_title("Chart of Force & Displacement vs. Time")
    ax_disp.grid(True, color="#444444", linestyle="--")

    ax_force.plot(
        state.history_timestamp_s,
        state.history_force_g,
        color="red",
        linewidth=1.8,
    )
    ax_force.set_facecolor("black")
    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel("Force (g)")
    ax_force.grid(True, color="#444444", linestyle="--")

    ax_xy.plot(
        state.history_displacement_mm,
        state.history_force_g,
        color="lime",
        marker="o",
        markersize=3,
        linewidth=1.5,
    )
    ax_xy.set_facecolor("black")
    ax_xy.set_title("X-Y Plot: Displacement (mm) vs. Force (g)")
    ax_xy.set_xlabel("Displacement (mm)")
    ax_xy.set_ylabel("Force (g)")
    ax_xy.grid(True, color="#666600", linestyle=":")

    plt.tight_layout()
    plt.show()