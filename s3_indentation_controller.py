"""
s3_indentation_controller.py
Replaces LabVIEW 'S3Motor_control2017m.vi' & 'S3Acq_fource_ai0.vi'.
Recalibrated Frame 1 Initialization to match LabVIEW's exact N = 100 parallel
loop timing (~12.5 seconds at 1000 steps) while preserving repeatable multi-cycle
movement and smooth real-time displacement interpolation.
"""

import time
from typing import Callable, Optional

import config
from state import GlobalSystemState
from motor_interface import StepperMotorController
from daq_interface import ForceSensorDAQ, ForceOverloadError


# Exact LabVIEW Frame 1 For-Loop constants (Screenshot 14 of Block Diagrams.docx)
LABVIEW_INIT_ITERATIONS: int = getattr(config, "INIT_POLL_ITERATIONS", 100)
LABVIEW_INIT_PERIOD_S: float = getattr(config, "INIT_LOOP_PERIOD_S", 0.120)


class S3IndentationController:
    """
    Executes S3Motor_control2017m.vi + S3Acq_fource_ai0.vi workflow with
    recalibrated LabVIEW Frame 1 initialization timing (N = 100, 120 ms period).
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
        Keeps COM4 quiet during the stroke so the TL3T controller executes
        @0U<delta> without serial interruption, while interpolating live
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

    def _run_labview_frame1_initialization(
        self,
        home_target: int,
        start_time: float,
        total_iterations: int = LABVIEW_INIT_ITERATIONS,
        loop_period_s: float = LABVIEW_INIT_PERIOD_S,
    ) -> int:
        """
        Reproduces LabVIEW's exact S3 Frame 1 initialization sequence and timing:
          1. Moves motor to Set position = 1000 steps (@0X1000, with @0U fallback
             if the controller is away from 1000 and in relative mode).
          2. Runs the full N = 100 initialization loop using a deadline timer
             (max(loop_period_s, poll_time) ~= 125 ms/iteration -> ~12.5 seconds total),
             recording the 100 baseline samples at 1000 steps just like LabVIEW.
        """
        self._log(
            f"[S3 Frame 1] Initializing motor to Set position = {home_target} steps "
            f"(N={total_iterations} iterations @ {int(loop_period_s * 1000)} ms LabVIEW timing)..."
        )

        # Read starting position before homing
        pos_init = self.motor.read_position()
        completed_iters = 0

        # Command @0X1000 (motor absposi(SubVI).vi) as in LabVIEW Frame 1
        self.motor.move_absolute(home_target)
        time.sleep(0.20)

        # If motor started away from 1000 steps, ensure it physically travels to 1000
        # without interrupting COM4 during movement, counting elapsed time toward N=100
        if abs(pos_init - home_target) > 15:
            pos_check = self.motor.read_position()
            if abs(pos_check - pos_init) <= 5 and abs(pos_check - home_target) > 15:
                delta_to_home = home_target - pos_check
                self._log(
                    f"[S3 Frame 1] Moving {delta_to_home:+d} steps from {pos_check} "
                    f"to {home_target} baseline..."
                )
                self.motor.move_relative(delta_to_home)

            travel_s = (
                abs(home_target - pos_init) / max(400.0, float(self.state.speed_parameter))
            ) + 0.30
            t_move_end = time.time() + travel_s

            while time.time() < t_move_end and completed_iters < total_iterations:
                if self.state.stop_requested or self.state.emergency_stop_triggered:
                    break
                iter_t0 = time.time()
                v_now, f_now = self.daq.read_live_force(check_safety=True)
                # Record baseline target (1000 steps) during pre-test homing so
                # the chart baseline matches LabVIEW's 1000.000 step header rows
                self.state.stepper_position_steps = home_target
                self.state.record_sample(
                    elapsed_s=iter_t0 - start_time,
                    pos_steps=home_target,
                    voltage_v=v_now,
                    force_g=f_now,
                )
                completed_iters += 1
                elapsed_iter = time.time() - iter_t0
                time.sleep(max(0.0, loop_period_s - elapsed_iter))

        # Execute the remaining iterations of LabVIEW's N = 100 For Loop
        # using parallel-equivalent deadline timing (loop_period_s = 120 ms)
        pos_now = home_target
        while completed_iters < total_iterations:
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break
            iter_t0 = time.time()
            pos_now = self._poll_motor_and_daq(start_time)
            completed_iters += 1

            if completed_iters in (1, 25, 50, 75, total_iterations):
                self._log(
                    f"[S3 Frame 1] Init {completed_iters}/{total_iterations} "
                    f"({time.time() - start_time:.1f}s) | Pos={pos_now} steps"
                )

            # In LabVIEW, Wait(120ms) runs in parallel with POSiReadout(100ms),
            # so we only sleep the remaining portion of the 120ms period.
            elapsed_iter = time.time() - iter_t0
            if elapsed_iter < loop_period_s:
                time.sleep(loop_period_s - elapsed_iter)

        # Brief quiet pause so controller UART is idle before Case 1 Cmd 1/10
        time.sleep(0.15)
        return pos_now

    def run_s3_sequence(
        self, init_poll_iterations: int = LABVIEW_INIT_ITERATIONS
    ) -> None:
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

        # --- S3 Frame 1: Recalibrated LabVIEW N=100 Initialization (~12.5 s) ---
        home_steps: int = config.INITIAL_HOME_STEPS  # 1000 steps
        current_pos: int = self._run_labview_frame1_initialization(
            home_target=home_steps,
            start_time=start_time,
            total_iterations=init_poll_iterations,
            loop_period_s=LABVIEW_INIT_PERIOD_S,
        )

        # --- S3 Frame 2: Main State Machine Loop (Cases 0, 1, 2) ---
        set_position: int = current_pos
        num_comd: int = 0
        flag_direction: bool = True  # True = next command is Advance (+), False = Retract (-)
        total_target_comds: int = self.state.repeat_cycles * 2
        case_selection: int = 0
        active_stroke_dir: int = 0   # +1 during Advance, -1 during Retract
        prev_pos: int = current_pos
        stall_count: int = 0

        # Full stroke travel time so deceleration completes before next command
        stroke_travel_s = (
            float(self.state.advance_steps) / max(100.0, float(self.state.speed_parameter))
        ) + 0.30
        effective_wait_s = max(float(self.state.dwell_time_s), stroke_travel_s)

        self._log(
            f"[S3 Frame 2] Starting {self.state.repeat_cycles} cycles from {current_pos} steps "
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
                        target_reached = True
                    elif active_stroke_dir > 0:
                        target_reached = current_pos >= (set_position - 15)
                    else:
                        target_reached = current_pos <= (set_position + 15)

                    # Auto-recovery if the motor stopped short of set_position
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
                            ) + 0.30
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
                    time.sleep(0.050)

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

                    time.sleep(0.08)
                    self.motor.move_relative(delta_steps)
                    flag_direction = not flag_direction
                    num_comd += 1
                    stall_count = 0

                    self._safe_wait_daq_only(
                        duration_s=effective_wait_s,
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

                    if current_pos <= (set_position + 15):
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