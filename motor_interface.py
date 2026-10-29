"""
motor_interface.py
Replaces LabVIEW Motor VIs using LabVIEW's exact per-SubVI lifecycle:
  - S2_Motor_Set speed2015.vi : "@0SI<speed>,\r\n" & "@0SI<speed>,<speed>,<speed>\r\n"
  - motor absposi(SubVI).vi   : "@0X<steps>\r\n" (NO trailing comma)
  - motorposi(SubVI).vi       : "@0U<steps>\r\n" (NO trailing comma)
  - POSiReadout.vi            : "a\r\n" -> wait 100ms -> read 10 bytes -> parse from index 1
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
    Hardware interface for the TL3T stepper motor controller on COM4 (9600 baud).
    Reproduces LabVIEW's exact SubVI open -> write/read -> close behavior.
    """

    def __init__(self, state: GlobalSystemState):
        self.state = state
        self.ser: Optional[serial.Serial] = None

    def _open_port(self) -> serial.Serial:
        """Opens COM4 cleanly with DTR/RTS handshaking disabled."""
        ser = serial.Serial()
        ser.port = self.state.com_port
        ser.baudrate = self.state.baud_rate
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = config.SERIAL_TIMEOUT_S
        ser.write_timeout = config.SERIAL_TIMEOUT_S
        ser.xonxoff = False
        ser.rtscts = False
        ser.dsrdtr = False
        ser.open()
        return ser

    def connect(self) -> None:
        """Verifies that the serial port can be opened cleanly."""
        try:
            ser = self._open_port()
            ser.close()
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

    def _visa_write_transaction(self, ascii_cmd: str, post_delay_s: float = 0.06) -> None:
        """
        Replaces LabVIEW's exact SubVI pattern:
          VISA Configure Serial Port (9600) -> VISA Write -> VISA Close
        """
        try:
            ser = self._open_port()
            try:
                ser.write(ascii_cmd.encode("ascii"))
                ser.flush()
                time.sleep(post_delay_s)
            finally:
                ser.close()
            time.sleep(0.04)
        except serial.SerialException as exc:
            raise MotorCommunicationError(
                f"Serial write failed while sending {ascii_cmd!r} on {self.state.com_port}: {exc}"
            ) from exc

    # =========================================================================
    # Stage 2: S2_Motor_Set speed2015.vi (Block Diagram Image 10)
    # =========================================================================
    def set_speed_s2(self, speed_steps_per_s: Optional[int] = None) -> str:
        """
        Sends "@0SI<speed>,\r\n" and "@0SI<speed>,<speed>,<speed>\r\n".
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

        cmd_s2 = f"@0SI{speed_int},\r\n"
        self._visa_write_transaction(cmd_s2)

        cmd_3axis = f"@0SI{speed_int},{speed_int},{speed_int}\r\n"
        self._visa_write_transaction(cmd_3axis)
        return cmd_s2

    # =========================================================================
    # SubVI 1: motor absposi(SubVI).vi (Block Diagram Image 2)
    # =========================================================================
    def move_absolute(self, target_steps: int) -> str:
        """
        Replaces motor absposi(SubVI).vi.
        Concatenates: "@0X" + dec_string(target_steps) + "\r\n" (NO comma).
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
        self._visa_write_transaction(cmd_str, post_delay_s=0.08)
        return cmd_str

    # =========================================================================
    # SubVI 2: motorposi(SubVI).vi (Block Diagram Image 4)
    # =========================================================================
    def move_relative(self, delta_steps: int) -> str:
        """
        Replaces motorposi(SubVI).vi.
        Concatenates: "@0U" + dec_string(delta_steps) + "\r\n" (NO comma).
        Does not block retraction based on estimated software position.
        """
        if self.state.emergency_stop_triggered:
            raise MotorSafetyLimitError("Movement blocked: Emergency Stop is active.")

        delta_int = int(delta_steps)
        if abs(delta_int) > config.MAX_SAFE_STEPS:
            raise MotorSafetyLimitError(
                f"Relative move {delta_int} steps exceeds maximum stroke limit "
                f"({config.MAX_SAFE_STEPS} steps)."
            )

        cmd_str = f"@0U{delta_int}\r\n"
        self._visa_write_transaction(cmd_str, post_delay_s=0.08)
        return cmd_str

    # =========================================================================
    # SubVI 3: POSiReadout.vi (Block Diagram Image 6)
    # =========================================================================
    def read_position(self) -> int:
        """
        Replaces POSiReadout.vi:
          Frame 0: VISA Configure Serial Port (9600) -> VISA Write("a\\r\\n")
          Frame 1: Wait 100 ms
          Right:   VISA Read(10 bytes) -> VISA Close -> Decimal String To Number (offset=1)
        """
        last_error = None
        for attempt in range(1, 4):
            try:
                ser = self._open_port()
                try:
                    ser.reset_input_buffer()
                    ser.write(b"a\r\n")
                    ser.flush()
                    time.sleep(config.POS_READOUT_WAIT_S)  # 100 ms wait
                    raw_bytes = ser.read(config.POS_READOUT_BYTES)
                finally:
                    ser.close()

                raw_text = raw_bytes.decode("ascii", errors="ignore").strip()
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
                time.sleep(0.06)

        raise MotorCommunicationError(
            f"POSiReadout failed on {self.state.com_port}: {last_error}"
        )

    def emergency_stop(self) -> None:
        """Halts the motor by commanding 0 relative movement and setting E-stop flag."""
        self.state.emergency_stop_triggered = True
        self.state.stop_requested = True
        try:
            self._visa_write_transaction("@0U0\r\n", post_delay_s=0.05)
        except Exception:
            pass