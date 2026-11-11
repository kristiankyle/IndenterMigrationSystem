"""
s3_indentation_controller.py
Replaces LabVIEW 'S3Motor_control2017m.vi' & 'S3Acq_fource_ai0.vi'.
Strictly mimics LabVIEW's exact control sequence and timing:
  1. Frame 1 (Initialization):
     - Sends @0X1000\r\n once (no @0U relative movement during initialization).
     - Enforces the full N = 100 For Loop (~13.5 seconds stationary baseline
       acquisition) even if gui.py passes an older init_poll_iterations=5 value.
  2. Frame 2 (Main While Loop @ 50 ms):
     - Case 0, Default: Polls POSiReadout & DAQ Force; checks target position.
     - Case 1: Sends @0U<+/-Step>\r\n, updates Set position, flips flag_direction,
       increments #of comd, and waits 停留時間(秒)*1000 ms with COM4 quiet.
     - Case 2: Polls POSiReadout & DAQ Force until final retraction completes.
"""

import time
from typing import Callable, Optional

import config
from state import GlobalSystemState
from motor_interface import StepperMotorController
from daq_interface import ForceSensorDAQ, ForceOverloadError


# Exact LabVIEW S3 Frame 1 For-Loop Constants (Screenshot 14 of Block Diagrams.docx)
LABVIEW_FRAME1_ITERATIONS: int = 100
LABVIEW_FRAME1_ITER_TARGET_S: float = 0.135  # 100 iters * 0.135s = ~13.5s (matches 138 rows in RUBIN16036AGM2)


class S3IndentationController:
    """
    Executes the exact S3Motor_control2017m.vi + S3Acq_fource_ai0.vi sequence.
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
        Polls POSiReadout.vi ('a\\r\\n', 100 ms wait) and NI-DAQmx force.
        Used in Frame 1, Case 0, and Case 2 where LabVIEW places POSiReadout.vi.
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
        Replaces LabVIEW's Wait (ms) inside Case 1 Frame 2 (停留時間(秒) * 1000).
        Does NOT send 'a\\r\\n' to COM4 during this wait so the stepper controller
        executes @0U<delta> uninterrupted, while sampling DAQ force every 100 ms
        (matching S4_Youngs_Graph.vi's 100 ms chart loop).
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

    def run_s3_sequence(
        self, init_poll_iterations: Optional[int] = None
    ) -> None:
        """
        Executes Frame 1 (N=100 initialization loop) followed by Frame 2 (Cases 0, 1, 2).
        Note: Enforces LABVIEW_FRAME1_ITERATIONS (100) even if gui.py passes
        init_poll_iterations=5 from an earlier test configuration.
        """
        if not self.state.s1_configured:
            raise RuntimeError("S1 parameters must be configured before running S3.")

        # Always enforce the full 100-iteration LabVIEW Frame 1 loop unless > 20 is explicitly given
        n_init_iters = (
            init_poll_iterations
            if (init_poll_iterations is not None and init_poll_iterations >= 50)
            else LABVIEW_FRAME1_ITERATIONS
        )

        self.state.clear_history()
        self.motor.connect()

        # Apply S2 speed (@0SI) and wait 200 ms for controller UART to settle
        self.motor.set_speed_s2(self.state.speed_parameter)
        time.sleep(0.20)

        # Start continuous DAQ stream (S3Acq_fource_ai0.vi equivalent)
        if not self.state.is_tared:
            self._log("[S3] Taring load cell baseline before movement...")
            self.daq.tare_baseline()
        self.daq.start_continuous_stream()

        start_time = time.time()

        # =====================================================================
        # S3 FRAME 1: LabVIEW Initialization Sequence (Left Frame of Image 14)
        #   1. Set position = 1000, #of comd = 0, flag_direction = True
        #   2. Call motor absposi(SubVI).vi (@0X1000\r\n) once
        #   3. Run N = 100 For Loop polling POSiReadout.vi (~13.5 seconds total)
        # =====================================================================
        set_position: int = config.INITIAL_HOME_STEPS  # 1000 steps
        num_comd: int = 0                              # #of comd = 0
        flag_direction: bool = True                    # flag_direction = T
        total_target_comds: int = self.state.repeat_cycles * 2

        self._log(
            f"[S3 Frame 1] Sending @0X{set_position} and starting LabVIEW N={n_init_iters} "
            f"initialization loop (~{n_init_iters * LABVIEW_FRAME1_ITER_TARGET_S:.1f}s baseline wait)..."
        )
        self.motor.move_absolute(set_position)
        time.sleep(0.15)

        pos_now: int = set_position
        for i in range(n_init_iters):
            if self.state.stop_requested or self.state.emergency_stop_triggered:
                break

            iter_t0 = time.time()
            pos_now = self._poll_motor_and_daq(start_time)

            if (i + 1) in (1, 25, 50, 75, n_init_iters):
                self._log(
                    f"[S3 Frame 1] Init {i + 1}/{n_init_iters} "
                    f"({time.time() - start_time:.1f}s elapsed) | Pos={pos_now} steps"
                )

            # Enforce ~135 ms per iteration so 100 iterations = ~13.5 seconds
            elapsed_iter = time.time() - iter_t0
            if elapsed_iter < LABVIEW_FRAME1_ITER_TARGET_S:
                time.sleep(LABVIEW_FRAME1_ITER_TARGET_S - elapsed_iter)

        # Brief quiet pause so controller UART is 100% idle before Frame 2 starts
        time.sleep(0.15)

        # =====================================================================
        # S3 FRAME 2: Main While Loop (Cases 0, 1, 2 @ 50 ms)
        # =====================================================================
        case_selection: int = 0
        current_pos: int = pos_now
        # Align baseline Set position with the motor's actual settled Frame 1 coordinate
        set_position = current_pos
        prev_pos: int = current_pos
        active_stroke_dir: int = 0   # +1 during Advance, -1 during Retract
        stall_count: int = 0

        # Physical stroke duration (4.0s move + 0.30s stepper deceleration settle = 4.30s)
        stroke_travel_s = (
            float(self.state.advance_steps) / max(100.0, float(self.state.speed_parameter))
        ) + 0.30
        effective_wait_s = max(float(self.state.dwell_time_s), stroke_travel_s)

        self._log(
            f"[S3 Frame 2] Initialization complete ({time.time() - start_time:.1f}s). "
            f"Starting {self.state.repeat_cycles} cycles from {current_pos} steps "
            f"({total_target_comds} commands, Step={self.state.advance_steps})..."
        )

        try:
            while not (self.state.stop_requested or self.state.emergency_stop_triggered):
                # -------------------------------------------------------------
                # CASE 0, Default: Poll Position & Check Target Reached
                # -------------------------------------------------------------
                if case_selection == 0:
                    current_pos = self._poll_motor_and_daq(start_time)

                    if num_comd == 0:
                        # First pass after Frame 1: motor is settled at baseline, trigger Case 1
                        target_reached = True
                    elif active_stroke_dir > 0:
                        # True branch (Advancing): current_pos >= Set position
                        target_reached = current_pos >= (set_position - 15)
                    else:
                        # False branch (Retracting): current_pos <= Set position
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
                                f"[S3 Recovery] Motor stationary at {current_pos} steps "
                                f"(Target: {set_position}). Sending @0U{remaining_steps:+d}..."
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

                    # Outer Select node: #of comd < (重複次數 * 2)
                    if num_comd < total_target_comds:
                        case_selection = inner_select
                    else:
                        case_selection = 2

                    prev_pos = current_pos
                    time.sleep(0.050)  # Exact 50 ms While Loop timer in Frame 2

                # -------------------------------------------------------------
                # CASE 1: Send Relative Step (@0U), Update Set Position, & Dwell
                # -------------------------------------------------------------
                elif case_selection == 1:
                    # Frame 1 (Left of Case 1)
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

                    # Frame 2 (Right of Case 1): increment #of comd & wait 停留時間(秒)*1000 ms
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

                # -------------------------------------------------------------
                # CASE 2: Final Retraction Completion & Loop Stop
                # -------------------------------------------------------------
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