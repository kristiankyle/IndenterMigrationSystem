"""
s3_indentation_controller.py
Replaces LabVIEW 'S3Motor_control2017m.vi' & 'S3Acq_fource_ai0.vi'.
Ensures repeatable multi-run operation without manual retraction by:
  1. Waiting for physical homing to complete in Frame 1 (using @0X1000 and fallback
     relative homing @0U if needed) before starting Frame 2.
  2. Keeping COM4 completely quiet during Case 1 stroke movement so @0U is never
     aborted by POSiReadout ('a\\r\\n').
  3. Waiting the full physical stroke travel time (steps / speed + 0.35s settle buffer)
     so the motor finishes decelerating before the next command is sent.
  4. Checking explicit stroke direction (+1 Advance vs -1 Retract) in Case 0 instead of
     (current_pos - prev_pos) >= 0 so missed retractions never accumulate forward steps.
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
        """Polls hardware position ('a\\r\\n') and DAQ force in Frame 1, Case 0, and Case 2."""
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
        Replaces LabVIEW's Wait (ms) inside Case 1 Frame 2.
        Does NOT send 'a\\r\\n' over COM4 during the stroke so the TL3T controller
        executes @0U<delta> without serial interruption, while interpolating live
        position for the real-time waveform and X-Y hysteresis plots.
        """
        t0 = time.time()
        end_time = t0 + duration_s
        total_delta = target_pos_steps - start_pos_steps
        direction = 1 if total_delta >= 0 else -1
        speed = max(1, self.state.speed_parameter)

        while time.time() < end_time:
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break

            now = time.time()
            dt = now - t0
            moved = min(abs(total_delta), int(round(speed * dt)))
            est_pos = start_pos_steps + (direction * moved)
            self.state.stepper_position_steps = est_pos

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

    def _home_to_baseline(self, home_target: int, start_time: float) -> int:
        """
        Ensures the motor physically returns to home_target (1000 steps) and comes
        to a complete stop before Frame 2 begins.
        """
        pos_now = self._poll_motor_and_daq(start_time)
        if abs(pos_now - home_target) <= 15:
            return pos_now

        self._log(
            f"[S3 Init] Homing motor from {pos_now} steps to {home_target} steps..."
        )
        self.motor.move_absolute(home_target)
        time.sleep(0.35)

        pos_check = self._poll_motor_and_daq(start_time)
        # If @0X did not start moving the motor toward home_target, use relative @0U
        if abs(pos_check - pos_now) <= 5 and abs(pos_check - home_target) > 15:
            delta_to_home = home_target - pos_check
            self._log(
                f"[S3 Init] Using relative homing (@0U{delta_to_home:+d}) to reach {home_target} steps..."
            )
            self.motor.move_relative(delta_to_home)
            travel_s = abs(delta_to_home) / max(400.0, float(self.state.speed_parameter))
            self._safe_wait_daq_only(
                duration_s=travel_s + 0.35,
                start_time=start_time,
                start_pos_steps=pos_check,
                target_pos_steps=home_target,
            )

        # Poll until motor arrives at home_target and stops moving
        prev_p = pos_check
        settled_reads = 0
        for _ in range(80):
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break
            p_curr = self._poll_motor_and_daq(start_time)
            if abs(p_curr - home_target) <= 15 and abs(p_curr - prev_p) <= 3:
                settled_reads += 1
                if settled_reads >= 2:
                    pos_now = p_curr
                    break
            elif abs(p_curr - prev_p) <= 2:
                settled_reads += 1
                if settled_reads >= 4:
                    pos_now = p_curr
                    break
            else:
                settled_reads = 0
            prev_p = p_curr
            time.sleep(0.120)

        # Extra quiet pause so the controller is completely idle before Cycle 1
        time.sleep(0.30)
        return self._poll_motor_and_daq(start_time)

    def run_s3_sequence(self, init_poll_iterations: int = 40) -> None:
        if not self.state.s1_configured:
            raise RuntimeError("S1 parameters must be configured before running S3.")

        self.state.clear_history()
        self.motor.connect()

        # Apply S2 speed and give the controller 200 ms to settle
        self.motor.set_speed_s2(self.state.speed_parameter)
        time.sleep(0.20)

        # Start continuous DAQ stream (S3Acq_fource_ai0.vi equivalent)
        if not self.state.is_tared:
            self._log("[S3] Taring load cell baseline before movement...")
            self.daq.tare_baseline()
        self.daq.start_continuous_stream()

        start_time = time.time()

        # --- S3 Frame 1: Complete Hardware Homing to 1000 Steps ---
        home_steps: int = config.INITIAL_HOME_STEPS  # 1000 steps
        current_pos: int = self._home_to_baseline(home_steps, start_time)

        # --- S3 Frame 2: Main State Machine Loop (Cases 0, 1, 2) ---
        set_position: int = current_pos
        num_comd: int = 0
        flag_direction: bool = True  # True = next command is Advance (+), False = Retract (-)
        total_target_comds: int = self.state.repeat_cycles * 2
        case_selection: int = 0
        active_stroke_dir: int = 0   # +1 during Advance, -1 during Retract
        prev_pos: int = current_pos
        stall_count: int = 0

        # Physical stroke time so we never cut off a move before deceleration finishes
        stroke_travel_s = (
            float(self.state.advance_steps) / max(100.0, float(self.state.speed_parameter))
        ) + 0.35
        effective_wait_s = max(float(self.state.dwell_time_s), stroke_travel_s)

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

                    if num_comd == 0:
                        # Ready to issue Cmd 1/N immediately from settled baseline
                        target_reached = True
                    elif active_stroke_dir > 0:
                        # Advancing stroke: must reach >= set_position (within 15 steps)
                        target_reached = current_pos >= (set_position - 15)
                    else:
                        # Retracting stroke: must reach <= set_position (within 15 steps)
                        target_reached = current_pos <= (set_position + 15)

                    # If the motor stopped short or missed a command, re-send the remaining delta
                    if not target_reached and num_comd > 0:
                        if abs(current_pos - prev_pos) <= 3:
                            stall_count += 1
                        else:
                            stall_count = 0

                        if stall_count >= 2:
                            remaining_steps = set_position - current_pos
                            self._log(
                                f"[S3 Auto-Retry] Motor at {current_pos} steps (Target: {set_position}). "
                                f"Sending @0U{remaining_steps:+d}..."
                            )
                            time.sleep(0.15)
                            self.motor.move_relative(remaining_steps)
                            retry_wait_s = (
                                abs(remaining_steps) / max(100.0, float(self.state.speed_parameter))
                            ) + 0.35
                            self._safe_wait_daq_only(
                                duration_s=retry_wait_s,
                                start_time=start_time,
                                start_pos_steps=current_pos,
                                target_pos_steps=set_position,
                            )
                            stall_count = 0
                            prev_pos = current_pos
                            continue
                    else:
                        stall_count = 0

                    inner_select = 1 if target_reached else 0

                    if num_comd < total_target_comds:
                        case_selection = inner_select
                    else:
                        case_selection = 2

                    prev_pos = current_pos
                    time.sleep(0.060)

                # =============================================================
                # CASE 1: Send Relative Step (@0U), Update Set Position, & Dwell
                # =============================================================
                elif case_selection == 1:
                    active_stroke_dir = 1 if flag_direction else -1
                    delta_steps = self.state.advance_steps * active_stroke_dir
                    start_pos = current_pos
                    set_position = current_pos + delta_steps

                    action_label = "Advancing (Load)" if flag_direction else "Retracting (Unload)"
                    self._log(
                        f"[S3 Case 1] Cmd {num_comd + 1}/{total_target_comds}: "
                        f"{action_label} {delta_steps:+d} steps -> Target: {set_position} steps"
                    )

                    # Brief pause after Case 0's POSiReadout so COM4 is ready for @0U
                    time.sleep(0.10)
                    self.motor.move_relative(delta_steps)
                    flag_direction = not flag_direction
                    num_comd += 1
                    stall_count = 0

                    # Wait full stroke duration with COM4 quiet (polling DAQ only)
                    self._safe_wait_daq_only(
                        duration_s=effective_wait_s,
                        start_time=start_time,
                        start_pos_steps=start_pos,
                        target_pos_steps=set_position,
                    )

                    case_selection = 0
                    time.sleep(0.080)

                # =============================================================
                # CASE 2: Final Retraction Completion & Loop Stop
                # =============================================================
                elif case_selection == 2:
                    current_pos = self._poll_motor_and_daq(start_time)

                    if current_pos <= (set_position + 15):
                        self._log(
                            f"[S3 Case 2] Final position reached ({current_pos} steps). "
                            "S3 sequence complete."
                        )
                        break

                    case_selection = 0
                    prev_pos = current_pos
                    time.sleep(0.060)

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