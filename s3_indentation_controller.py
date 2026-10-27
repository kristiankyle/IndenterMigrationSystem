"""
s3_indentation_controller.py
Replaces LabVIEW 'S3Motor_control2017m.vi' (Screenshots 13, 14, and Case 0/1/2).
Fixes serial command collision by keeping COM4 quiet during Case 1 Frame 2 wait
(matching LabVIEW's exact Case 1 block diagram) while still streaming live DAQ data.
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
         - Move absolute to 1000 (@0X1000\r\n)
         - Poll POSiReadout in N=100 loop (120ms timer) updating live Waveform Chart
      2. Frame 2 (Main While Loop @ 50ms interval):
         - Case 0, Default: Poll POSiReadout & DAQ Force; check if target reached.
         - Case 1: Frame 1 -> send @0U<delta>\r\n, Set position = Position + delta,
           invert flag_direction. Frame 2 -> #of comd += 1, wait 停留時間(秒) * 1000 ms
           (polling DAQ only so COM4 motion is never interrupted by 'a\r\n').
         - Case 2: Poll POSiReadout & DAQ Force until current_pos <= Set position (1000).
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
        """
        Used in Frame 1, Case 0, and Case 2 (where LabVIEW places POSiReadout.vi).
        Polls motor position ('a\\r\\n') and NI-DAQmx force.
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

    def _safe_wait_daq_only(
        self,
        duration_s: float,
        start_time: float,
        start_pos_steps: int,
        target_pos_steps: int,
    ) -> None:
        """
        Replaces LabVIEW's Wait (ms) inside Case 1 Frame 2 (image_55d07d.png).
        IMPORTANT: Does NOT send 'a\\r\\n' to COM4 during this wait so the TL3T
        stepper controller executes @0U<delta> without serial buffer interruption!
        Estimates live position from speed_parameter during the wait and then
        Case 0 verifies the exact hardware position via POSiReadout.vi immediately after.
        """
        t_wait_start = time.time()
        end_time = t_wait_start + duration_s
        delta_total = target_pos_steps - start_pos_steps
        direction = 1 if delta_total >= 0 else -1
        speed = max(1, self.state.speed_parameter)

        while time.time() < end_time:
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break

            now = time.time()
            dt = now - t_wait_start
            # Kinematic position estimate while motor is in flight during Frame 2 wait
            moved_steps = min(abs(delta_total), int(round(speed * dt)))
            est_pos = start_pos_steps + (direction * moved_steps)

            v_now, f_now = self.daq.read_live_force(check_safety=True)
            self.state.record_sample(
                elapsed_s=now - start_time,
                pos_steps=est_pos,
                voltage_v=v_now,
                force_g=f_now,
            )

            remaining = end_time - time.time()
            if remaining > 0:
                time.sleep(min(0.100, remaining))

    def run_s3_sequence(self, init_poll_iterations: int = 100) -> None:
        """
        Executes the full S3Motor_control2017m.vi + S3Acq_fource_ai0.vi workflow.
        """
        if not self.state.s1_configured:
            raise RuntimeError("S1 parameters must be configured before running S3.")

        self.state.clear_history()
        self.motor.connect()

        # Apply S2 speed before starting S3 movement
        self.motor.set_speed_s2(self.state.speed_parameter)
        time.sleep(0.15)

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
        # Allow controller 150 ms to start @0X1000 before polling 'a\r\n'
        time.sleep(0.15)

        settled_count = 0
        for i in range(init_poll_iterations):
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break
            pos_now = self._poll_motor_and_daq(start_time)
            if abs(pos_now - set_position) <= 2:
                settled_count += 1
                if settled_count >= 3:
                    self._log(f"[S3 Frame 1] Motor settled at {pos_now} steps (iter {i + 1}).")
                    break
            else:
                settled_count = 0
            time.sleep(0.120)

        # --- S3 Frame 2: Main State Machine Loop (Cases 0, 1, 2) ---
        case_selection: int = 0
        prev_pos: int = 0
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
                    current_pos = self._poll_motor_and_daq(start_time)

                    # Inner Case Structure selector: (current_pos - prev_pos) >= 0
                    if (current_pos - prev_pos) >= 0:
                        target_reached = current_pos >= set_position
                    else:
                        target_reached = current_pos <= set_position

                    inner_select = 1 if target_reached else 0

                    # Outer Select node: #of comd < (重複次數 * 2)
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
                    # Frame 1 (Left of Case 1 in image_55d07d.png)
                    direction_sign = 1 if flag_direction else -1
                    delta_steps = self.state.advance_steps * direction_sign
                    start_pos = current_pos
                    set_position = current_pos + delta_steps

                    action_label = "Advancing (Load)" if flag_direction else "Retracting (Unload)"
                    self._log(
                        f"[S3 Case 1] Cmd {num_comd + 1}/{total_target_comds}: "
                        f"{action_label} {delta_steps:+d} steps -> Target: {set_position} steps"
                    )

                    self.motor.move_relative(delta_steps)
                    flag_direction = not flag_direction

                    # Frame 2 (Right of Case 1 in image_55d07d.png):
                    # Wait 停留時間(秒)*1000 ms WITHOUT sending serial 'a\r\n'
                    num_comd += 1
                    wait_s = max(0.2, self.state.dwell_time_s)
                    self._safe_wait_daq_only(
                        duration_s=wait_s,
                        start_time=start_time,
                        start_pos_steps=start_pos,
                        target_pos_steps=set_position,
                    )

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
