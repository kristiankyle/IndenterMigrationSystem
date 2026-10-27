"""
s3_indentation_controller.py
Replaces LabVIEW 'S3Motor_control2017m.vi' (Screenshots 13, 14, and Case 0/1/2).
"""

import time
from typing import Callable, Optional

import config
from state import GlobalSystemState
from motor_interface import StepperMotorController
from daq_interface import ForceSensorDAQ, ForceOverloadError


class S3IndentationController:
    """
    Executes S3Motor_control2017m.vi + S3Acq_fource_ai0.vi workflow.
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

    def _poll_motor_and_daq(self, start_time: float) -> int:
        pos_now = self.motor.read_position()
        v_now, f_now = self.daq.read_live_force(check_safety=True)
        self.state.record_sample(
            elapsed_s=time.time() - start_time,
            pos_steps=pos_now,
            voltage_v=v_now,
            force_g=f_now,
        )
        return pos_now

    def _safe_wait_with_daq_polling(self, duration_s: float, start_time: float) -> None:
        """
        Replaces LabVIEW's Wait (ms) inside Case 1 Frame 2 (停留時間(秒) * 1000),
        polling DAQ every 100 ms while keeping COM4 quiet during movement.
        """
        end_time = time.time() + duration_s
        while time.time() < end_time:
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break
            v_now, f_now = self.daq.read_live_force(check_safety=True)
            self.state.record_sample(
                elapsed_s=time.time() - start_time,
                pos_steps=self.state.stepper_position_steps,
                voltage_v=v_now,
                force_g=f_now,
            )
            remaining = end_time - time.time()
            time.sleep(min(0.100, max(0.0, remaining)))

    def run_s3_sequence(self, init_poll_iterations: int = 20) -> None:
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

        # --- S3 Frame 1: Hardware Initialization ---
        set_position: int = config.INITIAL_HOME_STEPS  # 1000 steps
        num_comd: int = 0                              # #of comd = 0
        flag_direction: bool = True                    # flag_direction = T
        total_target_comds: int = self.state.repeat_cycles * 2

        self._log(
            f"[S3 Init] Moving to baseline Set position = {set_position} steps (@0X{set_position})..."
        )
        self.motor.move_absolute(set_position)

        for i in range(init_poll_iterations):
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break
            pos_now = self._poll_motor_and_daq(start_time)
            if pos_now == set_position and i >= 2:
                break
            time.sleep(0.120)

        # --- S3 Frame 2: Main State Machine Loop (Cases 0, 1, 2) ---
        case_selection: int = 0
        prev_pos: int = 0
        current_pos: int = self.state.stepper_position_steps
        # Align set_position to current starting position if motor is at 0
        if current_pos < set_position:
            set_position = current_pos

        self._log(
            f"[S3 Loop] Starting {self.state.repeat_cycles} cycles from {current_pos} steps "
            f"({total_target_comds} half-cycle commands, Step={self.state.advance_steps})..."
        )

        try:
            while not (self.state.stop_requested or self.state.emergency_stop_triggered):
                # =============================================================
                # CASE 0, Default: Poll Position & Check Target Reached
                # =============================================================
                if case_selection == 0:
                    current_pos = self._poll_motor_and_daq(start_time)

                    if (current_pos - prev_pos) >= 0:
                        target_reached = current_pos >= set_position
                    else:
                        target_reached = current_pos <= set_position

                    inner_select = 1 if target_reached else 0

                    if num_comd < total_target_comds:
                        case_selection = inner_select
                    else:
                        case_selection = 2

                    prev_pos = current_pos
                    time.sleep(0.050)

                # =============================================================
                # CASE 1: Send Relative Step (@0U), Update Set Position, & Dwell
                # =============================================================
                elif case_selection == 1:
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

                    num_comd += 1
                    if self.state.dwell_time_s > 0:
                        self._safe_wait_with_daq_polling(self.state.dwell_time_s, start_time)

                    case_selection = 0
                    time.sleep(0.050)

                # =============================================================
                # CASE 2: Final Retraction Completion & Loop Stop
                # =============================================================
                elif case_selection == 2:
                    current_pos = self._poll_motor_and_daq(start_time)

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