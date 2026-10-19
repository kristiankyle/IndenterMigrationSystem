"""
gui.py
Unified Operator GUI replacing the legacy LabVIEW S1 -> S4 Front Panels.
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import config
from daq_interface import DAQCommunicationError, ForceOverloadError, ForceSensorDAQ
from motor_interface import (
    MotorCommunicationError,
    MotorSafetyLimitError,
    StepperMotorController,
)
from s1_parameter_setting import (
    ParameterValidationError,
    calculate_and_set_s1_parameters,
)
from s3_indentation_controller import S3IndentationController
from s4_graph_and_export import (
    calculate_stiffness_and_modulus,
    export_s4_spreadsheets,
    plot_s4_graphs,
)
from state import GlobalSystemState


class IndenterOperatorGUI:
    """
    Complete operator interface for S1 Parameter Setup, S2 Motor Speed Configuration,
    Load Cell Tare/Calibration, Manual Motor Positioning, S3 Automated Indentation,
    Emergency Stop, and S4 Young's Modulus Graphing & Spreadsheet Export.
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Python Indenter Control System (S1 - S4 Replacement)")
        self.root.geometry("920x740")
        self.root.configure(padx=12, pady=12)

        # Shared system state & hardware controllers
        self.state = GlobalSystemState()
        self.motor = StepperMotorController(self.state)
        self.daq = ForceSensorDAQ(self.state)

        self.s3_thread: threading.Thread = None
        self.is_s3_running: bool = False
        self.is_live_monitoring: bool = False
        self.log_queue: queue.Queue = queue.Queue()

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start 100 ms UI refresh timer
        self.root.after(100, self._periodic_ui_refresh)

    def _build_ui(self) -> None:
        # --- Top Title Banner ---
        header = tk.Label(
            self.root,
            text="Mechanical Indenter Control & Force Acquisition System",
            font=("Helvetica", 16, "bold"),
        )
        header.pack(pady=(0, 8))

        # --- Digital Live Telemetry Display ---
        telemetry_frame = tk.Frame(self.root, bg="black", bd=4, relief="sunken")
        telemetry_frame.pack(fill=tk.X, pady=(0, 10))

        self.lbl_live_force = tk.Label(
            telemetry_frame,
            text="Force: 0.0 g",
            font=("Helvetica", 32, "bold"),
            fg="lime",
            bg="black",
        )
        self.lbl_live_force.pack(side=tk.LEFT, expand=True, pady=8)

        self.lbl_live_pos = tk.Label(
            telemetry_frame,
            text="Pos: 0 steps (0.000 mm)",
            font=("Helvetica", 22, "bold"),
            fg="#00d4ff",
            bg="black",
        )
        self.lbl_live_pos.pack(side=tk.LEFT, expand=True, pady=8)

        self.lbl_live_voltage = tk.Label(
            telemetry_frame,
            text="DAQ: 0.00000 V | Tare: Not Locked",
            font=("Consolas", 12),
            fg="white",
            bg="black",
        )
        self.lbl_live_voltage.pack(side=tk.RIGHT, padx=15, pady=8)

        # --- Middle 2-Column Container ---
        mid_frame = tk.Frame(self.root)
        mid_frame.pack(fill=tk.BOTH, expand=False, pady=4)

        left_col = tk.Frame(mid_frame)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

        right_col = tk.Frame(mid_frame)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(6, 0))

        # =====================================================================
        # LEFT COLUMN 1: Hardware Connection & Load Cell Calibration
        # =====================================================================
        hw_box = ttk.LabelFrame(left_col, text="1. Hardware & Load Cell Calibration", padding=10)
        hw_box.pack(fill=tk.X, pady=(0, 8))

        row0 = tk.Frame(hw_box)
        row0.pack(fill=tk.X, pady=2)
        tk.Label(row0, text="Motor Port:").pack(side=tk.LEFT)
        self.ent_com = tk.Entry(row0, width=8)
        self.ent_com.insert(0, config.DEFAULT_COM_PORT)
        self.ent_com.pack(side=tk.LEFT, padx=4)

        tk.Label(row0, text="DAQ Chan:").pack(side=tk.LEFT, padx=(8, 0))
        self.ent_daq = tk.Entry(row0, width=10)
        self.ent_daq.insert(0, config.DEFAULT_DAQ_CHANNEL)
        self.ent_daq.pack(side=tk.LEFT, padx=4)

        tk.Label(row0, text="Factor (g/V):").pack(side=tk.LEFT, padx=(8, 0))
        self.ent_factor = tk.Entry(row0, width=9)
        self.ent_factor.insert(0, f"{config.CALIBRATED_GRAMS_PER_VOLT:.2f}")
        self.ent_factor.pack(side=tk.LEFT, padx=4)

        cal_btn_row = tk.Frame(hw_box)
        cal_btn_row.pack(fill=tk.X, pady=(8, 2))

        self.btn_tare = tk.Button(
            cal_btn_row,
            text="Zero / Tare Baseline (Dev1/ai1)",
            bg="#e0e0e0",
            font=("Helvetica", 10, "bold"),
            command=self.action_tare_daq,
        )
        self.btn_tare.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_cal_500g = tk.Button(
            cal_btn_row,
            text="500g Reverse-Tare Calibrate",
            bg="#e0e0e0",
            font=("Helvetica", 10),
            command=self.action_calibrate_500g,
        )
        self.btn_cal_500g.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_live_monitor = tk.Button(
            hw_box,
            text="Start Live Force Stream (Pre-Test Check)",
            bg="#cce5ff",
            font=("Helvetica", 10, "bold"),
            command=self.action_toggle_live_stream,
        )
        self.btn_live_monitor.pack(fill=tk.X, pady=(6, 0))

        # =====================================================================
        # LEFT COLUMN 2: S1 Parameter Setting & S2 Speed Configuration
        # =====================================================================
        s1_box = ttk.LabelFrame(
            left_col,
            text="2. Stage S1 & S2: Test Parameters & Motor Speed",
            padding=10,
        )
        s1_box.pack(fill=tk.X, pady=(0, 8))

        grid_f = tk.Frame(s1_box)
        grid_f.pack(fill=tk.X)

        tk.Label(grid_f, text="前進距離 Advance (mm):").grid(row=0, column=0, sticky="w", pady=3)
        self.ent_advance_mm = tk.Entry(grid_f, width=10)
        self.ent_advance_mm.insert(0, str(config.DEFAULT_ADVANCE_MM))
        self.ent_advance_mm.grid(row=0, column=1, padx=6, pady=3)

        tk.Label(grid_f, text="上升時間 Rise Time (s):").grid(row=1, column=0, sticky="w", pady=3)
        self.ent_rise_s = tk.Entry(grid_f, width=10)
        self.ent_rise_s.insert(0, str(config.DEFAULT_RISE_TIME_S))
        self.ent_rise_s.grid(row=1, column=1, padx=6, pady=3)

        tk.Label(grid_f, text="停留時間 Dwell Time (s):").grid(row=2, column=0, sticky="w", pady=3)
        self.ent_dwell_s = tk.Entry(grid_f, width=10)
        self.ent_dwell_s.insert(0, str(config.DEFAULT_DWELL_TIME_S))
        self.ent_dwell_s.grid(row=2, column=1, padx=6, pady=3)

        tk.Label(grid_f, text="重複次數 Cycles:").grid(row=3, column=0, sticky="w", pady=3)
        self.ent_cycles = tk.Entry(grid_f, width=10)
        self.ent_cycles.insert(0, str(config.DEFAULT_REPEAT_CYCLES))
        self.ent_cycles.grid(row=3, column=1, padx=6, pady=3)

        tk.Label(grid_f, text="Force Cutoff Limit (g):").grid(row=4, column=0, sticky="w", pady=3)
        self.ent_max_force = tk.Entry(grid_f, width=10)
        self.ent_max_force.insert(0, str(config.MAX_SAFE_FORCE_GRAMS))
        self.ent_max_force.grid(row=4, column=1, padx=6, pady=3)

        self.lbl_s1_computed = tk.Label(
            s1_box,
            text="前進STEP: Not Set  |  速度參數: Not Set",
            font=("Consolas", 10, "bold"),
            fg="blue",
        )
        self.lbl_s1_computed.pack(pady=6)

        self.btn_s1_s2 = tk.Button(
            s1_box,
            text="Set UP (Run S1 Math & Send S2 Speed @0SI to Motor)",
            bg="#d4edda",
            font=("Helvetica", 10, "bold"),
            command=self.action_apply_s1_and_s2,
        )
        self.btn_s1_s2.pack(fill=tk.X)

        # =====================================================================
        # RIGHT COLUMN 1: Manual Motor Controls (SubVIs)
        # =====================================================================
        manual_box = ttk.LabelFrame(
            right_col,
            text="3. Manual Stepper Motor Control (COM4 SubVIs)",
            padding=10,
        )
        manual_box.pack(fill=tk.X, pady=(0, 8))

        pos_row = tk.Frame(manual_box)
        pos_row.pack(fill=tk.X, pady=3)

        self.btn_read_pos = tk.Button(
            pos_row,
            text="Read Position (POSiReadout 'a')",
            command=self.action_read_motor_pos,
        )
        self.btn_read_pos.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_home_1000 = tk.Button(
            pos_row,
            text="Move Home 1000 (@0X1000)",
            command=lambda: self.action_move_abs(config.INITIAL_HOME_STEPS),
        )
        self.btn_home_1000.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        jog_row = tk.Frame(manual_box)
        jog_row.pack(fill=tk.X, pady=4)
        tk.Label(jog_row, text="Jog Steps:").pack(side=tk.LEFT)
        self.ent_jog_steps = tk.Entry(jog_row, width=8)
        self.ent_jog_steps.insert(0, "400")
        self.ent_jog_steps.pack(side=tk.LEFT, padx=4)

        self.btn_jog_up = tk.Button(
            jog_row,
            text="Retract -Steps (@0U-)",
            command=lambda: self.action_jog(-1),
        )
        self.btn_jog_up.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_jog_down = tk.Button(
            jog_row,
            text="Advance +Steps (@0U+)",
            command=lambda: self.action_jog(1),
        )
        self.btn_jog_down.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # =====================================================================
        # RIGHT COLUMN 2: S3 Indentation Sequence & S4 Analysis
        # =====================================================================
        s3_box = ttk.LabelFrame(
            right_col,
            text="4. Stage S3 & S4: Automated Indentation & Analysis",
            padding=10,
        )
        s3_box.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        self.btn_start_s3 = tk.Button(
            s3_box,
            text="START S3 INDENTATION SEQUENCE",
            bg="#28a745",
            fg="white",
            font=("Helvetica", 12, "bold"),
            height=2,
            command=self.action_start_s3,
        )
        self.btn_start_s3.pack(fill=tk.X, pady=4)

        stop_row = tk.Frame(s3_box)
        stop_row.pack(fill=tk.X, pady=4)

        self.btn_stop_s3 = tk.Button(
            stop_row,
            text="Stop Sequence",
            bg="#ffc107",
            font=("Helvetica", 11, "bold"),
            command=self.action_request_stop,
        )
        self.btn_stop_s3.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))

        self.btn_estop = tk.Button(
            stop_row,
            text="EMERGENCY STOP",
            bg="#dc3545",
            fg="white",
            font=("Helvetica", 11, "bold"),
            command=self.action_emergency_stop,
        )
        self.btn_estop.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(4, 0))

        self.btn_s4_export = tk.Button(
            s3_box,
            text="Stage S4: Plot Young's Graph & Export Spreadsheets",
            bg="#cce5ff",
            font=("Helvetica", 11, "bold"),
            height=2,
            command=self.action_run_s4,
        )
        self.btn_s4_export.pack(fill=tk.X, pady=(8, 2))

        self.lbl_s4_summary = tk.Label(
            s3_box,
            text="Recorded Samples: 0 | Stiffness: -- g/mm | E: -- kPa",
            font=("Consolas", 10),
        )
        self.lbl_s4_summary.pack(pady=4)

        # --- Bottom Status & Error Log ---
        log_box = ttk.LabelFrame(self.root, text="System Status & Hardware Log", padding=6)
        log_box.pack(fill=tk.BOTH, expand=True)

        self.txt_log = tk.Text(log_box, height=8, font=("Consolas", 9), state=tk.DISABLED)
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        self.enqueue_log("System initialized. Configure S1/S2 and Tare Load Cell before running S3.")

    # =========================================================================
    # Logging & Periodic UI Refresh
    # =========================================================================
    def enqueue_log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{timestamp}] {message}")

    def _periodic_ui_refresh(self) -> None:
        # 1. Flush queued log messages
        while not self.log_queue.empty():
            msg = self.log_queue.get_nowait()
            self.txt_log.config(state=tk.NORMAL)
            self.txt_log.insert(tk.END, msg + "\n")
            self.txt_log.see(tk.END)
            self.txt_log.config(state=tk.DISABLED)

        # 2. If standalone live force monitor is active (outside S3), poll DAQ
        if self.is_live_monitoring and not self.is_s3_running:
            try:
                self.daq.read_live_force(check_safety=False)
            except Exception as exc:
                self.is_live_monitoring = False
                self.daq.stop_continuous_stream()
                self.btn_live_monitor.config(
                    text="Start Live Force Stream (Pre-Test Check)", bg="#cce5ff"
                )
                self.enqueue_log(f"[DAQ Stream Error] {exc}")

        # 3. Refresh digital telemetry labels from GlobalSystemState
        pos_steps = self.state.stepper_position_steps
        pos_mm = pos_steps / config.STEPS_PER_MM
        self.lbl_live_force.config(text=f"Force: {self.state.force_loadcell_g:.1f} g")
        self.lbl_live_pos.config(text=f"Pos: {pos_steps} steps ({pos_mm:.3f} mm)")

        tare_txt = f"{self.state.v_tare:.5f} V" if self.state.is_tared else "Not Tared"
        self.lbl_live_voltage.config(
            text=f"DAQ: {self.state.force_loadcell_v:.5f} V | Tare: {tare_txt}"
        )

        self.root.after(100, self._periodic_ui_refresh)

    def _sync_hw_settings_from_ui(self) -> None:
        self.state.com_port = self.ent_com.get().strip()
        self.state.daq_channel = self.ent_daq.get().strip()
        self.state.grams_per_volt = float(self.ent_factor.get().strip())
        config.MAX_SAFE_FORCE_GRAMS = float(self.ent_max_force.get().strip())

    # =========================================================================
    # Button Actions
    # =========================================================================
    def action_tare_daq(self) -> None:
        if self.is_s3_running:
            return
        try:
            self._sync_hw_settings_from_ui()
            was_streaming = self.is_live_monitoring
            if was_streaming:
                self.daq.stop_continuous_stream()
            v_tare = self.daq.tare_baseline()
            if was_streaming:
                self.daq.start_continuous_stream()
            self.enqueue_log(
                f"[DAQ Tare] Baseline locked at {v_tare:.6f} V on {self.state.daq_channel}."
            )
        except Exception as exc:
            self.enqueue_log(f"[DAQ Tare Error] {exc}")
            messagebox.showerror("DAQ Tare Error", str(exc))

    def action_calibrate_500g(self) -> None:
        if self.is_s3_running:
            return
        try:
            self._sync_hw_settings_from_ui()
            if not self.state.is_tared:
                messagebox.showwarning(
                    "Step 1 Required",
                    "First hang the 500g weight, press ZERO on the UAB-3000, "
                    "and click 'Zero / Tare Baseline'.\n\n"
                    "Then remove the 500g weight and click '500g Reverse-Tare Calibrate'.",
                )
                return
            factor = self.daq.calibrate_with_weight(499.0)
            self.ent_factor.delete(0, tk.END)
            self.ent_factor.insert(0, f"{factor:.2f}")
            self.enqueue_log(f"[Calibration] New conversion factor locked: {factor:.2f} g/V")
            messagebox.showinfo(
                "Calibration Complete",
                f"Calibrated Conversion Factor: {factor:.2f} g/V",
            )
        except Exception as exc:
            self.enqueue_log(f"[Calibration Error] {exc}")
            messagebox.showerror("Calibration Error", str(exc))

    def action_toggle_live_stream(self) -> None:
        if self.is_s3_running:
            return
        if not self.is_live_monitoring:
            try:
                self._sync_hw_settings_from_ui()
                if not self.state.is_tared:
                    self.daq.tare_baseline()
                self.daq.start_continuous_stream()
                self.is_live_monitoring = True
                self.btn_live_monitor.config(
                    text="Stop Live Force Stream", bg="#ffc107"
                )
                self.enqueue_log("[Live Monitor] Continuous DAQ force stream started.")
            except Exception as exc:
                self.enqueue_log(f"[Live Monitor Error] {exc}")
                messagebox.showerror("DAQ Error", str(exc))
        else:
            self.is_live_monitoring = False
            self.daq.stop_continuous_stream()
            self.btn_live_monitor.config(
                text="Start Live Force Stream (Pre-Test Check)", bg="#cce5ff"
            )
            self.enqueue_log("[Live Monitor] Continuous DAQ force stream stopped.")

    def action_apply_s1_and_s2(self) -> None:
        if self.is_s3_running:
            return
        try:
            self._sync_hw_settings_from_ui()
            dist_mm = float(self.ent_advance_mm.get().strip())
            rise_s = float(self.ent_rise_s.get().strip())
            dwell_s = float(self.ent_dwell_s.get().strip())
            cycles = int(self.ent_cycles.get().strip())

            res = calculate_and_set_s1_parameters(
                state=self.state,
                advance_distance_mm=dist_mm,
                rise_time_s=rise_s,
                dwell_time_s=dwell_s,
                repeat_cycles=cycles,
            )
            self.lbl_s1_computed.config(
                text=f"前進STEP: {res['前進STEP']} steps  |  速度參數: {res['速度參數']} steps/s"
            )
            self.enqueue_log(
                f"[S1 Configured] 前進STEP={res['前進STEP']}, "
                f"速度參數={res['速度參數']} steps/s, Cycles={res['重複次數']}"
            )

            # Immediately execute S2_Motor_Set speed2015.vi over COM4
            self.motor.connect()
            cmd = self.motor.set_speed_s2()
            pos = self.motor.read_position()
            self.enqueue_log(
                f"[S2 Configured] Sent {cmd.strip()} to {self.state.com_port} | Pos={pos} steps"
            )
        except (ParameterValidationError, MotorSafetyLimitError, MotorCommunicationError, ValueError) as exc:
            self.enqueue_log(f"[S1/S2 Error] {exc}")
            messagebox.showerror("S1 / S2 Configuration Error", str(exc))

    def action_read_motor_pos(self) -> None:
        if self.is_s3_running:
            return
        try:
            self._sync_hw_settings_from_ui()
            self.motor.connect()
            pos = self.motor.read_position()
            self.enqueue_log(f"[POSiReadout] Current motor position: {pos} steps")
        except Exception as exc:
            self.enqueue_log(f"[Motor Error] {exc}")
            messagebox.showerror("Motor Read Error", str(exc))

    def action_move_abs(self, target_steps: int) -> None:
        if self.is_s3_running:
            return
        try:
            self._sync_hw_settings_from_ui()
            self.state.emergency_stop_triggered = False
            self.motor.connect()
            cmd = self.motor.move_absolute(target_steps)
            self.enqueue_log(f"[Manual Move ABS] Sent {cmd.strip()}")
            self.root.after(800, self.action_read_motor_pos)
        except Exception as exc:
            self.enqueue_log(f"[Motor Move Error] {exc}")
            messagebox.showerror("Motor Move Error", str(exc))

    def action_jog(self, direction_sign: int) -> None:
        if self.is_s3_running:
            return
        try:
            self._sync_hw_settings_from_ui()
            self.state.emergency_stop_triggered = False
            self.motor.connect()
            self.motor.read_position()
            steps = int(self.ent_jog_steps.get().strip()) * direction_sign
            cmd = self.motor.move_relative(steps)
            self.enqueue_log(f"[Manual Jog] Sent {cmd.strip()}")
            self.root.after(600, self.action_read_motor_pos)
        except Exception as exc:
            self.enqueue_log(f"[Motor Jog Error] {exc}")
            messagebox.showerror("Motor Jog Error", str(exc))

    def action_start_s3(self) -> None:
        if self.is_s3_running:
            return
        try:
            self._sync_hw_settings_from_ui()
            if not self.state.s1_configured:
                self.action_apply_s1_and_s2()
                if not self.state.s1_configured:
                    return
        except Exception as exc:
            messagebox.showerror("Setup Error", str(exc))
            return

        if self.is_live_monitoring:
            self.is_live_monitoring = False
            self.daq.stop_continuous_stream()
            self.btn_live_monitor.config(
                text="Start Live Force Stream (Pre-Test Check)", bg="#cce5ff"
            )

        self.is_s3_running = True
        self.btn_start_s3.config(state=tk.DISABLED, text="S3 RUNNING...")

        self.s3_thread = threading.Thread(target=self._s3_worker, daemon=True)
        self.s3_thread.start()

    def _s3_worker(self) -> None:
        controller = S3IndentationController(
            state=self.state,
            motor=self.motor,
            daq=self.daq,
            status_callback=self.enqueue_log,
        )
        try:
            controller.run_s3_sequence()
            n_pts = len(self.state.history_position_steps)
            metrics = calculate_stiffness_and_modulus(self.state)
            self.enqueue_log(
                f"[S3 Complete] Recorded {n_pts} samples. "
                f"Stiffness={metrics['stiffness_g_per_mm']:.2f} g/mm"
            )
            self.root.after(0, self._update_s4_summary_label)
        except ForceOverloadError as exc:
            self.enqueue_log(f"[FORCE OVERLOAD CUTOFF] {exc}")
            self.root.after(0, lambda: messagebox.showerror("Force Overload Cutoff", str(exc)))
        except Exception as exc:
            self.enqueue_log(f"[S3 Execution Error] {exc}")
            self.root.after(0, lambda: messagebox.showerror("S3 Error", str(exc)))
        finally:
            self.is_s3_running = False
            self.root.after(
                0,
                lambda: self.btn_start_s3.config(
                    state=tk.NORMAL, text="START S3 INDENTATION SEQUENCE"
                ),
            )

    def _update_s4_summary_label(self) -> None:
        n_pts = len(self.state.history_position_steps)
        metrics = calculate_stiffness_and_modulus(self.state)
        self.lbl_s4_summary.config(
            text=(
                f"Recorded Samples: {n_pts} | "
                f"Stiffness: {metrics['stiffness_g_per_mm']:.2f} g/mm | "
                f"E: {metrics['youngs_modulus_kpa']:.2f} kPa"
            )
        )

    def action_request_stop(self) -> None:
        self.state.stop_requested = True
        self.enqueue_log("[Operator Stop] Stop requested. Halting sequence...")

    def action_emergency_stop(self) -> None:
        self.motor.emergency_stop()
        self.enqueue_log("[EMERGENCY STOP] Motor halted immediately by operator!")
        messagebox.showwarning(
            "EMERGENCY STOP TRIGGERED",
            "Motor movement halted immediately.\n"
            "Verify indenter clearance before commanding any new movement.",
        )

    def action_run_s4(self) -> None:
        if not self.state.history_position_steps:
            messagebox.showwarning("No Data", "Run an S3 Indentation Sequence first.")
            return
        try:
            raw_file, csv_file = export_s4_spreadsheets(self.state)
            self._update_s4_summary_label()
            self.enqueue_log(
                f"[S4 Exported] LabVIEW Raw: {raw_file} | Calibrated CSV: {csv_file}"
            )
            plot_s4_graphs(self.state)
        except Exception as exc:
            self.enqueue_log(f"[S4 Error] {exc}")
            messagebox.showerror("S4 Export/Plot Error", str(exc))

    def on_close(self) -> None:
        self.state.stop_requested = True
        self.is_live_monitoring = False
        self.daq.stop_continuous_stream()
        self.motor.disconnect()
        self.root.destroy()