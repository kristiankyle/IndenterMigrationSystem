"""
run_s1_to_s4_test.py
Complete sequential hardware test: S1 -> S2 -> S3 -> S4.
"""

from state import GlobalSystemState
from s1_parameter_setting import calculate_and_set_s1_parameters
from motor_interface import StepperMotorController
from daq_interface import ForceSensorDAQ
from s3_indentation_controller import S3IndentationController
from s4_graph_and_export import (
    export_s4_spreadsheets,
    calculate_stiffness_and_modulus,
    plot_s4_graphs,
)


def main():
    print("==========================================================")
    print("  COMPLETE S1 -> S4 HARDWARE VERIFICATION SEQUENCE        ")
    print("==========================================================\n")

    state = GlobalSystemState()
    motor = StepperMotorController(state)
    daq = ForceSensorDAQ(state)

    try:
        # --- STAGE 1: S1 Parameter Setting ---
        print("[STAGE 1] Configuring S1 Parameters (Safe 1-Cycle Test: 1.0 mm, 2s rise, 2s dwell)...")
        s1_res = calculate_and_set_s1_parameters(
            state=state,
            advance_distance_mm=1.0,  # 1600 steps (safe short stroke for first test)
            rise_time_s=2.0,          # 800 steps/s
            dwell_time_s=2.0,         # 2.0 s dwell
            repeat_cycles=1,          # 1 full load/unload cycle (2 commands)
        )
        print(f" -> S1 Globals Locked: Step={s1_res['前進STEP']}, Speed={s1_res['速度參數']} steps/s\n")

        # --- STAGE 2: S2 Motor Speed Setup ---
        print("[STAGE 2] Connecting to COM4 and configuring motor speed...")
        motor.connect()
        cmd = motor.set_speed_s2()
        print(f" -> S2 Sent: {cmd.strip()} | Current Pos: {motor.read_position()} steps\n")

        # --- STAGE 3: S3 Motor Control & Force Acquisition ---
        input(
            "[STAGE 3] Ensure indenter path is clear and UAB-3000 is ZEROED (0.0).\n"
            "Press ENTER to start S3 Indentation Sequence..."
        )
        daq.tare_baseline()
        s3 = S3IndentationController(state=state, motor=motor, daq=daq)
        s3.run_s3_sequence()

        # --- STAGE 4: S4 Graph & Spreadsheet Export ---
        print("\n[STAGE 4] Exporting data and generating S4 graphs...")
        raw_file, csv_file = export_s4_spreadsheets(state)
        metrics = calculate_stiffness_and_modulus(state)
        print(f" -> LabVIEW Raw DBL File : {raw_file}")
        print(f" -> Calibrated CSV File  : {csv_file}")
        print(f" -> Stiffness Slope      : {metrics['stiffness_g_per_mm']:.2f} g/mm")
        plot_s4_graphs(state)

        print("\n[SUCCESS] Full S1 -> S4 sequence completed and verified!")

    except KeyboardInterrupt:
        print("\n[ABORTED] User pressed Ctrl+C. Stopping motor...")
        motor.emergency_stop()
    except Exception as exc:
        print(f"\n[ERROR] Sequence halted: {exc}")
    finally:
        daq.stop_continuous_stream()
        motor.disconnect()


if __name__ == "__main__":
    main()