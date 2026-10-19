"""
s4_graph_and_export.py
Replaces LabVIEW 'S4_Youngs_Graph.vi' (Images 15, 16, 17 in Block Diagrams.docx).
Exports both the exact LabVIEW 2-column DBL spreadsheet and calibrated CSV,
computes stiffness / Young's Modulus, and plots the Time Chart and X-Y Plot.
"""

import csv
from datetime import datetime
from typing import Dict, Tuple
import numpy as np
import matplotlib.pyplot as plt

from state import GlobalSystemState


def export_s4_spreadsheets(
    state: GlobalSystemState, base_filename: str = "indentation_run"
) -> Tuple[str, str]:
    """
    1. Writes the exact LabVIEW 'Write To Spreadsheet File (DBL).vi' transposed
       2-column tab-separated file: [步進馬達位置, Fource_LoadCell (V)].
    2. Writes a full calibrated CSV file with mm, Volts, grams, and Newtons.
    """
    if not state.history_position_steps:
        raise ValueError("No recorded S3 data available to export in S4.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_lv_file = f"{base_filename}_{timestamp}_labview_raw.txt"
    calibrated_csv_file = f"{base_filename}_{timestamp}_calibrated.csv"

    # 1. Exact LabVIEW Write To Spreadsheet File (DBL).vi (Transposed 2 columns)
    with open(raw_lv_file, mode="w", encoding="utf-8", newline="") as f_lv:
        for pos_steps, volt_v in zip(
            state.history_position_steps, state.history_voltage_v
        ):
            f_lv.write(f"{float(pos_steps):.6f}\t{volt_v:.6f}\n")

    # 2. Full Calibrated Engineering CSV
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
        for t_s, steps, disp_mm, volt_v, force_g in zip(
            state.history_timestamp_s,
            state.history_position_steps,
            state.history_displacement_mm,
            state.history_voltage_v,
            state.history_force_g,
        ):
            force_n = (force_g / 1000.0) * 9.80665
            writer.writerow([
                f"{t_s:.4f}",
                f"{steps}",
                f"{disp_mm:.5f}",
                f"{volt_v:.6f}",
                f"{force_g:.3f}",
                f"{force_n:.5f}",
            ])

    return raw_lv_file, calibrated_csv_file


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
        return {"stiffness_g_per_mm": 0.0, "stiffness_n_per_mm": 0.0, "youngs_modulus_kpa": 0.0}

    # Linear regression slope dF/dh
    slope_g_mm, _ = np.polyfit(disp_mm, force_g, 1)
    slope_n_mm, _ = np.polyfit(disp_mm, force_n, 1)

    # Effective Young's Modulus (MPa -> kPa): 1 N/mm^2 = 1 MPa = 1000 kPa
    e_mpa = ((1.0 - poisson_ratio ** 2) / (2.0 * indenter_radius_mm)) * slope_n_mm
    e_kpa = float(e_mpa * 1000.0)

    return {
        "stiffness_g_per_mm": float(slope_g_mm),
        "stiffness_n_per_mm": float(slope_n_mm),
        "youngs_modulus_kpa": e_kpa,
    }


def plot_s4_graphs(state: GlobalSystemState) -> None:
    """
    Reproduces both graphs from S4_Youngs_Graph.vi Front Panel (Images 15 & 16):
      - Left/Top: Chart of Force & Displacement vs. Time
      - Right/Bottom: X-Y Plot (Displacement vs. Force)
    """
    fig, (ax_disp, ax_force, ax_xy) = plt.subplots(3, 1, figsize=(8, 9))
    fig.suptitle("S4_Youngs_Graph: Force & Displacement Analysis", fontsize=14, fontweight="bold")

    # 1. Displacement vs. Time (Matches upper plot of Image 15)
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

    # 2. Force vs. Time (Matches lower plot of Image 15)
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

    # 3. X-Y Plot: Displacement vs. Force (Matches Image 16)
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