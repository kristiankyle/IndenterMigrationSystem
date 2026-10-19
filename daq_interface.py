"""
daq_interface.py
Replaces LabVIEW 'S3Acq_fource_ai0.vi' and provides 500g Reverse-Tare calibration.
Communicates with NI USB-6218 via nidaqmx without modifying existing drivers.
"""

import time
from typing import Optional, Tuple
import numpy as np
import nidaqmx
from nidaqmx.constants import (
    AcquisitionType,
    READ_ALL_AVAILABLE,
    TerminalConfiguration,
)

import config
from state import GlobalSystemState


class DAQCommunicationError(RuntimeError):
    """Raised when NI-DAQmx hardware communication fails."""
    pass


class ForceOverloadError(RuntimeError):
    """Raised when measured force exceeds safe load-cell limits."""
    pass


class ForceSensorDAQ:
    """
    Hardware interface for Load Cell -> UAB-3000 (0-10V) -> NI USB-6218 (Dev1/ai1).
    """

    def __init__(self, state: GlobalSystemState):
        self.state = state
        self.continuous_task: Optional[nidaqmx.Task] = None
        self._last_voltage_v: float = 0.0

    def read_static_voltage(self, num_samples: int = config.DAQ_BLOCK_SIZE) -> float:
        """
        Takes a hardware-clocked finite block average (default 500 samples @ 1000 Hz)
        to eliminate static noise during baseline taring and calibration.
        """
        try:
            with nidaqmx.Task() as task:
                task.ai_channels.add_ai_voltage_chan(
                    self.state.daq_channel,
                    terminal_config=TerminalConfiguration.RSE,
                    min_val=config.DAQ_MIN_VOLTAGE,
                    max_val=config.DAQ_MAX_VOLTAGE,
                )
                task.timing.cfg_samp_clk_timing(
                    rate=config.DAQ_SAMPLE_RATE_HZ,
                    sample_mode=AcquisitionType.FINITE,
                    samps_per_chan=num_samples,
                )
                raw_data = task.read(
                    number_of_samples_per_channel=num_samples,
                    timeout=5.0,
                )
                v_mean = float(np.mean(raw_data))
                self._last_voltage_v = v_mean
                return v_mean
        except nidaqmx.DaqError as exc:
            raise DAQCommunicationError(
                f"NI-DAQmx failed on channel {self.state.daq_channel}: {exc}"
            ) from exc

    def tare_baseline(self) -> float:
        """
        Locks in the 0.0g baseline voltage (v_tare) and updates GlobalSystemState.
        """
        v_tare = self.read_static_voltage()
        self.state.v_tare = v_tare
        self.state.is_tared = True
        self.state.force_loadcell_v = v_tare
        self.state.force_loadcell_g = 0.0
        return v_tare

    def calibrate_with_weight(self, reference_weight_g: float = 499.0) -> float:
        """
        Computes grams_per_volt from a loaded voltage reading after tare_baseline().
        Formula: grams_per_volt = reference_weight_g / (v_load - v_tare)
        """
        if not self.state.is_tared:
            raise DAQCommunicationError("Run tare_baseline() (Step 1) before calibrating.")

        time.sleep(0.5)
        v_load = self.read_static_voltage()
        v_diff = v_load - self.state.v_tare

        if v_diff < 0.0005:
            raise DAQCommunicationError(
                f"Voltage difference too small ({v_diff:.6f} V). "
                "Ensure UAB-3000 was zeroed with the 500g weight hanging, "
                "and the weight was removed before Step 2."
            )

        factor = float(reference_weight_g) / v_diff
        self.state.grams_per_volt = factor
        return factor

    def start_continuous_stream(self) -> None:
        """
        Starts a continuous hardware-buffered acquisition task (500,000 samples)
        for S3 and S4 real-time operation without buffer overflow (-200279).
        """
        self.stop_continuous_stream()
        try:
            self.continuous_task = nidaqmx.Task()
            self.continuous_task.ai_channels.add_ai_voltage_chan(
                self.state.daq_channel,
                terminal_config=TerminalConfiguration.RSE,
                min_val=config.DAQ_MIN_VOLTAGE,
                max_val=config.DAQ_MAX_VOLTAGE,
            )
            self.continuous_task.timing.cfg_samp_clk_timing(
                rate=config.DAQ_SAMPLE_RATE_HZ,
                sample_mode=AcquisitionType.CONTINUOUS,
                samps_per_chan=config.DAQ_CONTINUOUS_BUFFER,
            )
            # Explicitly start task before any READ_ALL_AVAILABLE call (prevents -200983)
            self.continuous_task.start()
        except nidaqmx.DaqError as exc:
            self.stop_continuous_stream()
            raise DAQCommunicationError(
                f"Failed to start continuous DAQ stream on {self.state.daq_channel}: {exc}"
            ) from exc

    def read_live_force(self, check_safety: bool = True) -> Tuple[float, float]:
        """
        Flushes all available samples from the continuous buffer (zero lag),
        updates GlobalSystemState.force_loadcell_v & force_loadcell_g,
        and checks force overload limits.
        Returns: (voltage_v, force_g)
        """
        if self.continuous_task is None:
            v_now = self.read_static_voltage(num_samples=100)
        else:
            try:
                raw_samples = self.continuous_task.read(
                    number_of_samples_per_channel=READ_ALL_AVAILABLE,
                    timeout=2.0,
                )
                if raw_samples and len(raw_samples) > 0:
                    v_now = float(np.mean(raw_samples))
                    self._last_voltage_v = v_now
                else:
                    v_now = self._last_voltage_v
            except nidaqmx.DaqError as exc:
                raise DAQCommunicationError(
                    f"DAQ stream read error on {self.state.daq_channel}: {exc}"
                ) from exc

        force_g = (v_now - self.state.v_tare) * self.state.grams_per_volt
        self.state.force_loadcell_v = v_now
        self.state.force_loadcell_g = force_g

        if check_safety:
            if force_g >= config.ABSOLUTE_MAX_FORCE_GRAMS:
                self.state.emergency_stop_triggered = True
                raise ForceOverloadError(
                    f"CRITICAL OVERLOAD: Force reached {force_g:.1f} g "
                    f"(Hard limit: {config.ABSOLUTE_MAX_FORCE_GRAMS:.1f} g)!"
                )
            if force_g >= config.MAX_SAFE_FORCE_GRAMS:
                self.state.emergency_stop_triggered = True
                raise ForceOverloadError(
                    f"Force cutoff reached: {force_g:.1f} g "
                    f"(Safety limit: {config.MAX_SAFE_FORCE_GRAMS:.1f} g)."
                )

        return v_now, force_g

    def stop_continuous_stream(self) -> None:
        """Stops and closes the continuous NI-DAQmx task cleanly."""
        if self.continuous_task is not None:
            try:
                self.continuous_task.stop()
            except Exception:
                pass
            try:
                self.continuous_task.close()
            except Exception:
                pass
            finally:
                self.continuous_task = None


if __name__ == "__main__":
    print("==========================================================")
    print("  STAGE 3A: DAQ & Load Cell Verification (S3Acq_fource)   ")
    print("==========================================================\n")

    state = GlobalSystemState()
    daq = ForceSensorDAQ(state)

    try:
        input("Ensure indenter is empty and UAB-3000 is ZEROED (0.0), then press ENTER...")
        v_zero = daq.tare_baseline()
        print(f" -> Baseline Locked: {v_zero:.6f} V (Factor: {state.grams_per_volt:.2f} g/V)\n")

        print("Streaming live force for 5 seconds (gently push UP on indenter tip)...")
        daq.start_continuous_stream()
        t0 = time.time()
        while time.time() - t0 < 5.0:
            time.sleep(0.1)
            v_cur, f_cur = daq.read_live_force(check_safety=True)
            print(
                f"   Fource_LoadCell (V): {v_cur:.5f} V  |  Force: {f_cur:>7.1f} g    ",
                end="\r",
                flush=True,
            )
        print("\n\n[SUCCESS] DAQ interface verified.")
    except Exception as exc:
        print(f"\n[FAILURE] DAQ Test Error: {exc}")
    finally:
        daq.stop_continuous_stream()