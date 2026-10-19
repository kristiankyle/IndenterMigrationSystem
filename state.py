"""
state.py
Thread-safe replacement for LabVIEW's '2015myGlobal 1.vi'.
Holds all global parameters and real-time measurement arrays across S1 -> S4.
"""

import threading
from dataclasses import dataclass, field
from typing import List
import config


@dataclass
class GlobalSystemState:
    """
    Mirrors the exact global variables in 2015myGlobal 1.vi plus calibration state.
    """
    # --- S1 Operator Inputs ---
    advance_distance_mm: float = config.DEFAULT_ADVANCE_MM   # 前進距離(mm)
    rise_time_s: float = config.DEFAULT_RISE_TIME_S          # 上升時間(秒)
    dwell_time_s: float = config.DEFAULT_DWELL_TIME_S        # 停留時間(秒)
    repeat_cycles: int = config.DEFAULT_REPEAT_CYCLES        # 重複次數

    # --- S1 Calculated Globals (2015myGlobal 1.vi) ---
    advance_steps: int = 0                                   # 前進STEP (I32)
    speed_parameter: int = 0                                 # 速度參數 (I32)
    s1_configured: bool = False

    # --- Hardware Connection Settings ---
    com_port: str = config.DEFAULT_COM_PORT
    baud_rate: int = config.DEFAULT_BAUD_RATE
    daq_channel: str = config.DEFAULT_DAQ_CHANNEL

    # --- Load Cell Calibration State ---
    v_tare: float = 0.0
    grams_per_volt: float = config.CALIBRATED_GRAMS_PER_VOLT
    is_tared: bool = False

    # --- S3 Live Global Variables (2015myGlobal 1.vi) ---
    stepper_position_steps: int = 0                          # 步進馬達位置 (I32)
    force_loadcell_v: float = 0.0                            # Fource_LoadCell (V) (DBL)
    force_loadcell_g: float = 0.0                            # Converted Force (grams)

    # --- S4 Recorded Time-Series Arrays ---
    history_timestamp_s: List[float] = field(default_factory=list)
    history_position_steps: List[int] = field(default_factory=list)
    history_displacement_mm: List[float] = field(default_factory=list)
    history_voltage_v: List[float] = field(default_factory=list)
    history_force_g: List[float] = field(default_factory=list)

    # --- Process Control Flags ---
    stop_requested: bool = False
    emergency_stop_triggered: bool = False

    def __post_init__(self):
        self._lock = threading.Lock()

    def update_s1_globals(
        self,
        advance_distance_mm: float,
        rise_time_s: float,
        dwell_time_s: float,
        repeat_cycles: int,
        advance_steps: int,
        speed_parameter: int,
    ) -> None:
        """Thread-safe write of S1 parameters into 2015myGlobal 1.vi state."""
        with self._lock:
            self.advance_distance_mm = advance_distance_mm
            self.rise_time_s = rise_time_s
            self.dwell_time_s = dwell_time_s
            self.repeat_cycles = repeat_cycles
            self.advance_steps = advance_steps
            self.speed_parameter = speed_parameter
            self.s1_configured = True

    def record_sample(
        self,
        elapsed_s: float,
        pos_steps: int,
        voltage_v: float,
        force_g: float,
    ) -> None:
        """Thread-safe update of live globals and S4 history arrays."""
        with self._lock:
            self.stepper_position_steps = pos_steps
            self.force_loadcell_v = voltage_v
            self.force_loadcell_g = force_g

            self.history_timestamp_s.append(elapsed_s)
            self.history_position_steps.append(pos_steps)
            self.history_displacement_mm.append(pos_steps / config.STEPS_PER_MM)
            self.history_voltage_v.append(voltage_v)
            self.history_force_g.append(force_g)

    def clear_history(self) -> None:
        """Clears recorded arrays before starting a new S3/S4 test."""
        with self._lock:
            self.history_timestamp_s.clear()
            self.history_position_steps.clear()
            self.history_displacement_mm.clear()
            self.history_voltage_v.clear()
            self.history_force_g.clear()
            self.stop_requested = False
            self.emergency_stop_triggered = False