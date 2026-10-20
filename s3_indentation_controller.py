"""
s3_indentation_controller.py
Replaces LabVIEW 'S3Motor_control2017m.vi' (Screenshots 13, 14, and Case 0/1/2).
Coordinates the exact Frame 1 (N=100 initialization loop) and Frame 2 (Case 0/1/2
state machine) while continuously streaming Position and Force to the real-time charts.
"""

import time
from typing import Callable, Optional

import config
from state import GlobalSystemState
from motor_interface import StepperMotorController
from daq_interface import ForceSensorDAQ, ForceOverloadError


class S3IndentationController:
    """
    Executes the exact S3Motor_control2017m.vi sequence:
      1. Frame 1 (Initialization):
         - Set position = 1000, #of comd = 0, flag_direction = True
         - Move absolute to 1000 (@0X1000\\r\\n)
         - Poll POSiReadout in N=100 loop (120ms timer) updating live Waveform Chart
           until motor settles at 1000 steps.
      2. Frame 2 (Main While Loop @ 50ms interval):
         - Case 0, Default: Poll POSiReadout & DAQ Force; update live Waveform Chart;
           check if target reached (>= Set position when advancing, <= Set position
           when retracting).
         - Case 1: Frame 1 -> delta = Step * (+1 if flag_direction else -1),
           send @0U<delta>\\r\\n, Set position = Position + delta, invert flag_direction.
           Frame 2 -> #of comd += 1, wait 停留時間(秒) * 1000 ms while polling live DAQ & Position.
         - Case 2: Poll POSiReadout & DAQ Force until current_pos <= Set position (1000),
           then stop loop.
    """

    def __init__(
        self,
        state: GlobalSystemState,
        motor: StepperMotorController,
        daq: ForceSensorDAQ,
        status_callback: Optional[Callable[[str], None]] = None,
    ):
        self.state = state
        self.motor = motor
        self.daq = daq
        self.status_callback = status_callback

    def _log(self, message: str) -> None:
        if self.status_callback:
            self.status_callback(message)
        else:
            print(message)

    def _poll_and_record_live(self, start_time: float) -> int:
        """
        Polls POSiReadout.vi and NI-DAQmx force and appends a live point
        so the real-time GUI Waveform Chart and X-Y Plot update continuously.
        """
        pos_now = self.motor.read_position()
        v_now, f_now = self.daq.read_live_force(check_safety=True)
        self.state.record_sample(
            elapsed_s=time.time() - start_time,
            pos_steps=pos_now,
            voltage_v=v_now,
            force_g=f_now,
        )
        return pos_now

    def _safe_wait_with_live_polling(self, duration_s: float, start_time: float) -> None:
        """
        Executes LabVIEW's Wait (ms) inside Case 1 Frame 2 (停留時間(秒) * 1000)
        while polling Position and DAQ Force every 100 ms so the real-time
        Waveform Chart continues scrolling smoothly during movement/dwell.
        """
        end_time = time.time() + duration_s
        while time.time() < end_time:
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break
            self._poll_and_record_live(start_time)
            remaining = end_time - time.time()
            if remaining > 0:
                time.sleep(min(0.080, remaining))

    def run_s3_sequence(self, init_poll_iterations: int = 100) -> None:
        """
        Executes the full S3Motor_control2017m.vi + S3Acq_fource_ai0.vi workflow.
        Matches LabVIEW's N=100 Frame 1 loop so the motor always reaches 1000 steps.
        """
        if not self.state.s1_configured:
            raise RuntimeError("S1 parameters must be configured before running S3.")

        self.state.clear_history()
        self.motor.connect()

        # Apply S2 speed before starting S3 movement
        self.motor.set_speed_s2(self.state.speed_parameter)

        # Start continuous DAQ stream (S3Acq_fource_ai0.vi equivalent)
        if not self.state.is_tared:
            self._log("[S3] Taring load cell baseline before movement...")
            self.daq.tare_baseline()
        self.daq.start_continuous_stream()

        start_time = time.time()

        # --- S3 Frame 1: Hardware Initialization (Left Frame of Screenshot 14) ---
        set_position: int = config.INITIAL_HOME_STEPS  # 1000 steps
        num_comd: int = 0                              # #of comd = 0
        flag_direction: bool = True                    # flag_direction = T
        total_target_comds: int = self.state.repeat_cycles * 2

        self._log(
            f"[S3 Frame 1] Moving to baseline Set position = {set_position} steps (@0X{set_position})..."
        )
        self.motor.move_absolute(set_position)

        # LabVIEW Frame 1 For Loop (N=100, 120ms wait): Poll POSiReadout & plot live chart
        settled_count = 0
        for i in range(init_poll_iterations):
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break
            pos_now = self._poll_and_record_live(start_time)
            if abs(pos_now - set_position) <= 2:
                settled_count += 1
                # Once physically settled at 1000 steps for 4 consecutive reads, proceed
                if settled_count >= 4:
                    self._log(f"[S3 Frame 1] Motor settled at {pos_now} steps (iter {i + 1}).")
                    break
            else:
                settled_count = 0
            time.sleep(0.120)  # Exact 120 ms timer in S3 Frame 1

        # --- S3 Frame 2: Main State Machine Loop (Cases 0, 1, 2) ---
        case_selection: int = 0
        prev_pos: int = 0  # Shift register initialized to 0 on left border of While Loop
        current_pos: int = self.state.stepper_position_steps

        self._log(
            f"[S3 Frame 2] Starting {self.state.repeat_cycles} cycles "
            f"({total_target_comds} half-cycle commands, Step={self.state.advance_steps})..."
        )

        try:
            while not (self.state.stop_requested or self.state.emergency_stop_triggered):
                # =============================================================
                # CASE 0, Default: Poll Position & Check Target Reached
                # =============================================================
                if case_selection == 0:
                    current_pos = self._poll_and_record_live(start_time)

                    # Inner Case Structure selector: (current_pos - prev_pos) >= 0
                    if (current_pos - prev_pos) >= 0:
                        # True branch: current_pos >= Set position
                        target_reached = current_pos >= set_position
                    else:
                        # False branch: current_pos <= Set position
                        target_reached = current_pos <= set_position

                    inner_select = 1 if target_reached else 0

                    # Outer Select node: #of comd < (重複次數 * 2)
                    if num_comd < total_target_comds:
                        case_selection = inner_select
                    else:
                        case_selection = 2

                    prev_pos = current_pos
                    time.sleep(0.050)  # Exact 50 ms While Loop timer

                # =============================================================
                # CASE 1: Send Relative Step (@0U), Update Set Position, & Dwell
                # =============================================================
                elif case_selection == 1:
                    # Frame 1 (Left of Case 1)
                    direction_sign = 1 if flag_direction else -1
                    delta_steps = self.state.advance_steps * direction_sign
                    set_position = current_pos + delta_steps

                    action_label = "Advancing (Load)" if flag_direction else "Retracting (Unload)"
                    self._log(
                        f"[S3 Case 1] Cmd {num_comd + 1}/{total_target_comds}: "
                        f"{action_label} {delta_steps:+d} steps -> Target: {set_position} steps"
                    )

                    self.motor.move_relative(delta_steps)
                    flag_direction = not flag_direction

                    # Frame 2 (Right of Case 1): increment #of comd & wait 停留時間(秒)*1000 ms
                    num_comd += 1
                    if self.state.dwell_time_s > 0:
                        self._safe_wait_with_live_polling(self.state.dwell_time_s, start_time)

                    case_selection = 0
                    time.sleep(0.050)

                # =============================================================
                # CASE 2: Final Retraction Completion & Loop Stop
                # =============================================================
                elif case_selection == 2:
                    current_pos = self._poll_and_record_live(start_time)

                    # Stop condition in Case 2: current_pos <= Set position
                    if current_pos <= set_position:
                        self._log(
                            f"[S3 Case 2] Final position reached ({current_pos} steps). "
                            "S3 sequence complete."
                        )
                        break

                    case_selection = 0
                    prev_pos = current_pos
                    time.sleep(0.050)

        except ForceOverloadError as exc:
            self._log(
                f"[EMERGENCY RETRACT] {exc} Returning indenter to {config.INITIAL_HOME_STEPS} steps!"
            )
            try:
                self.state.emergency_stop_triggered = False
                self.motor.move_absolute(config.INITIAL_HOME_STEPS)
                self.state.emergency_stop_triggered = True
            except Exception:
                pass
            raise
        finally:
            self.daq.stop_continuous_stream()