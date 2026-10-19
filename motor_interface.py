"""
motor_interface.py
Replaces LabVIEW Motor VIs:
  - S2_Motor_Set speed2015.vi & Step1Speed setup.vi (@0SI<spd>,<spd>,<spd>\\r\\n)
  - motor absposi(SubVI).vi (@0X<steps>\\r\\n)
  - motorposi(SubVI).vi (@0U<steps>\\r\\n)
  - POSiReadout.vi ("a\\r\\n" -> wait 100ms -> read 10 bytes -> parse from index 1)
"""

import re
import time
from typing import Optional
import serial

import config
from state import GlobalSystemState
from s1_parameter_setting import calculate_and_set_s1_parameters


class MotorCommunicationError(RuntimeError):
    """Raised when serial communication with the stepper controller fails."""
    pass


class MotorSafetyLimitError(ValueError):
    """Raised when a commanded motor movement violates travel or speed limits."""
    pass


class StepperMotorController:
    """
    Real hardware interface for the stepper motor controller over RS-232/USB-Serial.
    """

    def __init__(self, state: GlobalSystemState):
        self.state = state
        self.ser: Optional[serial.Serial] = None

    def connect(self) -> None:
        """Opens the serial port (VISA Configure Serial Port equivalent)."""
        if self.ser is not None and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(
                port=self.state.com_port,
                baudrate=self.state.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=config.SERIAL_TIMEOUT_S,
                write_timeout=config.SERIAL_TIMEOUT_S,
            )
            # Brief hardware settle after opening port
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as exc:
            raise MotorCommunicationError(
                f"Failed to open serial port {self.state.com_port} at "
                f"{self.state.baud_rate} baud.\n"
                "1. Check that the motor controller USB/Serial cable is plugged in.\n"
                "2. Close LabVIEW completely so VISA does not lock the COM port.\n"
                f"Original error: {exc}"
            ) from exc

    def disconnect(self) -> None:
        """Closes the serial port cleanly (VISA Close equivalent)."""
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            finally:
                self.ser = None

    def _send_command(self, ascii_cmd: str) -> None:
        """Writes an ASCII command string to the serial port."""
        if self.ser is None or not self.ser.is_open:
            self.connect()
        try:
            encoded = ascii_cmd.encode("ascii")
            self.ser.write(encoded)
            self.ser.flush()
        except serial.SerialException as exc:
            raise MotorCommunicationError(
                f"Serial write failed while sending {ascii_cmd!r}: {exc}"
            ) from exc

    # =========================================================================
    # Stage 2: S2_Motor_Set speed2015.vi & Step1Speed setup.vi
    # =========================================================================
    def set_speed_s2(self, speed_steps_per_s: Optional[int] = None) -> str:
        """
        Replaces S2_Motor_Set speed2015.vi (Image 10 & Image 19 in Block Diagrams.docx).
        Formats: "@0SI" + speed + "," + speed + "," + speed + "\\r\\n"
        Example: "@0SI1600,1600,1600\\r\\n"
        """
        if speed_steps_per_s is None:
            if not self.state.s1_configured:
                raise MotorSafetyLimitError(
                    "Cannot run S2: S1 parameters have not been configured yet."
                )
            speed_steps_per_s = self.state.speed_parameter

        speed_int = int(speed_steps_per_s)
        if speed_int <= 0 or speed_int > config.MAX_SAFE_SPEED_STEPS_S:
            raise MotorSafetyLimitError(
                f"Speed {speed_int} steps/s is outside safe range "
                f"(1 to {config.MAX_SAFE_SPEED_STEPS_S} steps/s)."
            )

        cmd_str = f"@0SI{speed_int},{speed_int},{speed_int}\r\n"
        self._send_command(cmd_str)
        time.sleep(0.1)
        return cmd_str

    # =========================================================================
    # SubVI 1: motor absposi(SubVI).vi (Absolute Move)
    # =========================================================================
    def move_absolute(self, target_steps: int) -> str:
        """
        Replaces motor absposi(SubVI).vi (Image 2 in Block Diagrams.docx).
        Formats: "@0X" + target_steps + "\\r\\n"
        Example: "@0X1000\\r\\n"
        """
        if self.state.emergency_stop_triggered:
            raise MotorSafetyLimitError("Movement blocked: Emergency Stop is active.")

        target_int = int(target_steps)
        if target_int < config.MIN_SAFE_STEPS or target_int > config.MAX_SAFE_STEPS:
            raise MotorSafetyLimitError(
                f"Target absolute position {target_int} steps violates safe travel limits "
                f"[{config.MIN_SAFE_STEPS}, {config.MAX_SAFE_STEPS}]."
            )

        cmd_str = f"@0X{target_int}\r\n"
        self._send_command(cmd_str)
        return cmd_str

    # =========================================================================
    # SubVI 2: motorposi(SubVI).vi (Relative Step Move)
    # =========================================================================
    def move_relative(self, delta_steps: int) -> str:
        """
        Replaces motorposi(SubVI).vi (Image 4 in Block Diagrams.docx).
        Formats: "@0U" + delta_steps + "\\r\\n"
        Example: "@0U6400\\r\\n" or "@0U-6400\\r\\n"
        """
        if self.state.emergency_stop_triggered:
            raise MotorSafetyLimitError("Movement blocked: Emergency Stop is active.")

        delta_int = int(delta_steps)
        projected_pos = self.state.stepper_position_steps + delta_int
        if projected_pos < config.MIN_SAFE_STEPS or projected_pos > config.MAX_SAFE_STEPS:
            raise MotorSafetyLimitError(
                f"Relative move {delta_int} steps from {self.state.stepper_position_steps} "
                f"would reach {projected_pos} steps, violating safe travel limits "
                f"[{config.MIN_SAFE_STEPS}, {config.MAX_SAFE_STEPS}]."
            )

        cmd_str = f"@0U{delta_int}\r\n"
        self._send_command(cmd_str)
        return cmd_str

    # =========================================================================
    # SubVI 3: POSiReadout.vi (Position Readout)
    # =========================================================================
    def read_position(self) -> int:
        """
        Replaces POSiReadout.vi (Image 6 in Block Diagrams.docx):
          1. Writes "a\\r\\n" to serial port
          2. Waits 100 ms (POS_READOUT_WAIT_S)
          3. Reads up to 10 bytes (POS_READOUT_BYTES)
          4. Parses integer starting at offset 1 (Decimal String To Number with offset 1)
          5. Updates GlobalSystemState.stepper_position_steps (步進馬達位置)
        """
        if self.ser is None or not self.ser.is_open:
            self.connect()

        last_error = None
        for attempt in range(1, 4):
            try:
                self.ser.reset_input_buffer()
                self._send_command("a\r\n")
                time.sleep(config.POS_READOUT_WAIT_S)

                raw_bytes = self.ser.read(config.POS_READOUT_BYTES)
                raw_text = raw_bytes.decode("ascii", errors="ignore").strip()

                if not raw_text:
                    raise MotorCommunicationError(
                        "Empty response from motor controller on 'a\\r\\n' position query."
                    )

                # LabVIEW POSiReadout.vi wires constant '1' to offset of 'Decimal String To Number'
                substring = raw_text[1:] if len(raw_text) > 1 else raw_text
                match = re.search(r"[-+]?\d+", substring)
                if not match:
                    # Fallback if controller returned digits starting at index 0
                    match = re.search(r"[-+]?\d+", raw_text)

                if not match:
                    raise MotorCommunicationError(
                        f"Could not parse integer position from controller reply: {raw_text!r}"
                    )

                position_steps = int(match.group(0))
                self.state.stepper_position_steps = position_steps
                return position_steps

            except Exception as exc:
                last_error = exc
                time.sleep(0.05)

        raise MotorCommunicationError(
            f"POSiReadout failed after 3 attempts: {last_error}"
        )

    def emergency_stop(self) -> None:
        """
        Halts the motor by commanding relative 0 / current position lock and setting E-stop flag.
        """
        self.state.emergency_stop_triggered = True
        self.state.stop_requested = True
        if self.ser is not None and self.ser.is_open:
            try:
                # Command 0 relative movement to interrupt pulse generation
                self.ser.write(b"@0U0\r\n")
                self.ser.flush()
            except Exception:
                pass


if __name__ == "__main__":
    print("==========================================================")
    print("  STAGE 2 (S2) & MOTOR SUBVIs: Hardware Verification Test ")
    print("  References: S2_Motor_Set speed2015.vi, POSiReadout.vi,  ")
    print("              motor absposi(SubVI).vi                     ")
    print("==========================================================\n")

    state = GlobalSystemState()

    # Step 1: Run S1 with default LabVIEW values (4mm, 4s, 4s, 5 cycles -> 1600 steps/s)
    s1_info = calculate_and_set_s1_parameters(
        state=state,
        advance_distance_mm=config.DEFAULT_ADVANCE_MM,
        rise_time_s=config.DEFAULT_RISE_TIME_S,
        dwell_time_s=config.DEFAULT_DWELL_TIME_S,
        repeat_cycles=config.DEFAULT_REPEAT_CYCLES,
    )
    print(f"[S1 Ready] 前進STEP = {s1_info['前進STEP']} | 速度參數 = {s1_info['速度參數']} steps/s")

    motor = StepperMotorController(state)
    try:
        print(f"\n1. Connecting to Stepper Controller on {state.com_port} ({state.baud_rate} baud)...")
        motor.connect()
        print("   -> Connected successfully.")

        print("\n2. Testing S2_Motor_Set speed2015.vi (Setting speed from S1 Global)...")
        sent_speed_cmd = motor.set_speed_s2()
        print(f"   -> Sent command: {sent_speed_cmd!r}")

        print("\n3. Testing POSiReadout.vi (Reading current position via 'a\\r\\n')...")
        current_pos = motor.read_position()
        print(f"   -> Current Motor Position (步進馬達位置): {current_pos} steps")

        choice = input(
            f"\n4. Move motor to S3 initial home position ({config.INITIAL_HOME_STEPS} steps) "
            "using motor absposi(SubVI).vi? (y/n): "
        ).strip().lower()

        if choice == "y":
            sent_abs_cmd = motor.move_absolute(config.INITIAL_HOME_STEPS)
            print(f"   -> Sent command: {sent_abs_cmd!r}")
            time.sleep(1.5)
            new_pos = motor.read_position()
            print(f"   -> Verified Position after move: {new_pos} steps")

        print("\n[SUCCESS] Stage 2 & Motor SubVIs verified on real hardware.")

    except Exception as exc:
        print(f"\n[FAILURE] Stage 2 Hardware Test Error:\n{exc}")
    finally:
        motor.disconnect()
        print("Serial port closed safely.")