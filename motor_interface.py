"""
motor_interface.py
Replaces LabVIEW Motor VIs using LabVIEW's exact per-SubVI lifecycle:
  VISA Configure Serial Port -> VISA Write (/ Read) -> VISA Close
References:
  - S2_Motor_Set speed2015.vi (@0SI<speed>,\r\n)
  - motor absposi(SubVI).vi (@0X<steps>\r\n)
  - motorposi(SubVI).vi (@0U<steps>\r\n)
  - POSiReadout.vi ("a\r\n" -> wait 100ms -> read 10 bytes -> parse from index 1)
"""

import re
import time
from typing import Optional
import serial

import config
from state import GlobalSystemState


class MotorCommunicationError(RuntimeError):
    """Raised when serial communication with the stepper controller fails."""
    pass


class MotorSafetyLimitError(ValueError):
    """Raised when a commanded motor movement violates travel or speed limits."""
    pass


class StepperMotorController:
    """
    Real hardware interface for the TL3T stepper motor controller on COM4 (9600 baud).
    Reproduces LabVIEW's exact SubVI open -> write/read -> close behavior.
    """

    def __init__(self, state: GlobalSystemState):
        self.state = state
        self.ser: Optional[serial.Serial] = None

    def connect(self) -> None:
        """Verifies that the serial port can be opened cleanly."""
        try:
            with serial.Serial(
                port=self.state.com_port,
                baudrate=self.state.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=config.SERIAL_TIMEOUT_S,
                write_timeout=config.SERIAL_TIMEOUT_S,
            ) as _:
                pass
        except serial.SerialException as exc:
            raise MotorCommunicationError(
                f"Failed to open serial port {self.state.com_port} at "
                f"{self.state.baud_rate} baud.\n"
                "1. Check that the motor controller USB/Serial cable is plugged in.\n"
                "2. Check that the motor controller power switch is ON.\n"
                "3. Close LabVIEW completely so VISA does not lock the COM port.\n"
                f"Original error: {exc}"
            ) from exc

    def disconnect(self) -> None:
        """Closes any open serial handle."""
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            finally:
                self.ser = None

    def _visa_write_transaction(self, ascii_cmd: str) -> None:
        """
        Replaces LabVIEW's exact SubVI pattern:
          VISA Configure Serial Port (9600) -> VISA Write -> VISA Close
        """
        try:
            with serial.Serial(
                port=self.state.com_port,
                baudrate=self.state.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=config.SERIAL_TIMEOUT_S,
                write_timeout=config.SERIAL_TIMEOUT_S,
            ) as ser:
                ser.write(ascii_cmd.encode("ascii"))
                ser.flush()
                # Wait briefly before VISA Close so UART finishes shifting all bits out
                time.sleep(0.05)
        except serial.SerialException as exc:
            raise MotorCommunicationError(
                f"Serial write failed while sending {ascii_cmd!r} on {self.state.com_port}: {exc}"
            ) from exc

    # =========================================================================
    # Stage 2: S2_Motor_Set speed2015.vi (Block Diagram Image 10)
    # =========================================================================
    def set_speed_s2(self, speed_steps_per_s: Optional[int] = None) -> str:
        """
        Replaces S2_Motor_Set speed2015.vi (Image 10 in Block Diagrams.docx):
        Concatenates: "@0SI" + dec_string(速度參數) + "," + "\r" + "\n"
        Also sends "@0SI<spd>,<spd>,<spd>\r\n" from Speed setup.vi (Image 18) for compatibility.
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

        # Exact S2_Motor_Set speed2015.vi Block Diagram string (Image 10): "@0SI1600,\r\n"
        cmd_s2 = f"@0SI{speed_int},\r\n"
        self._visa_write_transaction(cmd_s2)
        time.sleep(0.05)

        # Also send 3-parameter form from Speed setup.vi (Image 18)
        cmd_3axis = f"@0SI{speed_int},{speed_int},{speed_int}\r\n"
        self._visa_write_transaction(cmd_3axis)
        time.sleep(0.05)
        return cmd_s2

    # =========================================================================
    # SubVI 1: motor absposi(SubVI).vi (Block Diagram Image 2)
    # =========================================================================
    def move_absolute(self, target_steps: int) -> str:
        """
        Replaces motor absposi(SubVI).vi (Image 2 in Block Diagrams.docx).
        Concatenates: "@0X" + dec_string(前進STEP) + "\r" + "\n"
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
        self._visa_write_transaction(cmd_str)
        return cmd_str

    # =========================================================================
    # SubVI 2: motorposi(SubVI).vi (Block Diagram Image 4)
    # =========================================================================
    def move_relative(self, delta_steps: int) -> str:
        """
        Replaces motorposi(SubVI).vi (Image 4 in Block Diagrams.docx).
        Concatenates: "@0U" + dec_string(前進STEP) + "\r" + "\n"
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
        self._visa_write_transaction(cmd_str)
        return cmd_str

    # =========================================================================
    # SubVI 3: POSiReadout.vi (Block Diagram Image 6)
    # =========================================================================
    def read_position(self) -> int:
        """
        Replaces POSiReadout.vi (Image 6 in Block Diagrams.docx):
          Frame 0: VISA Configure Serial Port (9600) -> VISA Write("a\\r\\n")
          Frame 1: Wait 100 ms
          Right:   VISA Read(10 bytes) -> VISA Close -> Decimal String To Number (offset=1)
        """
        last_error = None
        for attempt in range(1, 4):
            try:
                with serial.Serial(
                    port=self.state.com_port,
                    baudrate=self.state.baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=config.SERIAL_TIMEOUT_S,
                    write_timeout=config.SERIAL_TIMEOUT_S,
                ) as ser:
                    ser.write(b"a\r\n")
                    ser.flush()
                    time.sleep(config.POS_READOUT_WAIT_S)  # 100 ms wait (Frame 1)
                    raw_bytes = ser.read(config.POS_READOUT_BYTES)  # Read up to 10 bytes

                raw_text = raw_bytes.decode("ascii", errors="ignore").strip()

                # Exact LabVIEW Decimal String To Number with offset = 1:
                # If string has no digits or is empty, LabVIEW returns default 0
                substring = raw_text[1:] if len(raw_text) > 1 else raw_text
                match = re.search(r"[-+]?\d+", substring)
                if not match:
                    match = re.search(r"[-+]?\d+", raw_text)

                if not match:
                    raise MotorCommunicationError(
                        f"No position digits in controller reply: {raw_bytes!r}"
                    )

                position_steps = int(match.group(0))
                self.state.stepper_position_steps = position_steps
                return position_steps

            except Exception as exc:
                last_error = exc
                time.sleep(0.05)

        raise MotorCommunicationError(
            f"POSiReadout failed on {self.state.com_port}: {last_error}"
        )

    def emergency_stop(self) -> None:
        """Halts the motor by commanding 0 relative movement and setting E-stop flag."""
        self.state.emergency_stop_triggered = True
        self.state.stop_requested = True
        try:
            self._visa_write_transaction("@0U0\r\n")
        except Exception:
            pass