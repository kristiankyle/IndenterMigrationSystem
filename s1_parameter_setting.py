"""
s1_parameter_setting.py
Replaces LabVIEW 'S1_parametetsSetting2017.vi'.
Validates inputs, computes 前進STEP and 速度參數, and writes to GlobalSystemState.
"""

import config
from state import GlobalSystemState


class ParameterValidationError(ValueError):
    """Raised when S1 parameters violate physical or mathematical safety limits."""
    pass


def calculate_and_set_s1_parameters(
    state: GlobalSystemState,
    advance_distance_mm: float,
    rise_time_s: float,
    dwell_time_s: float,
    repeat_cycles: int,
) -> dict:
    """
    Executes the exact block diagram logic of S1_parametetsSetting2017.vi:
      1. 前進Steps (I32) = int(round(前進距離(mm) * 1600))
      2. 馬達速度設定參數 (I32) = int(前進Steps // 上升時間(秒))
      3. Stores 前進STEP, 速度參數, 停留時間(秒), 重複次數 in GlobalSystemState.
    """
    # --- 1. Validate Input Types and Safety Bounds ---
    if advance_distance_mm <= 0:
        raise ParameterValidationError(
            f"前進距離(mm) must be > 0 (got {advance_distance_mm})."
        )
    if advance_distance_mm > config.MAX_SAFE_ADVANCE_MM:
        raise ParameterValidationError(
            f"前進距離(mm) ({advance_distance_mm} mm) exceeds safety limit "
            f"({config.MAX_SAFE_ADVANCE_MM} mm)."
        )
    if rise_time_s <= 0:
        raise ParameterValidationError(
            f"上升時間(秒) must be > 0 to prevent division by zero (got {rise_time_s})."
        )
    if dwell_time_s < 0:
        raise ParameterValidationError(
            f"停留時間(秒) cannot be negative (got {dwell_time_s})."
        )
    if int(repeat_cycles) < 1:
        raise ParameterValidationError(
            f"重複次數 must be at least 1 cycle (got {repeat_cycles})."
        )

    # --- 2. LabVIEW Block Diagram Math (Image 8 of Block Diagrams.docx) ---
    # Multiply 前進距離(mm) by 1600 and convert to 32-bit integer (I32)
    advance_steps = int(round(float(advance_distance_mm) * config.STEPS_PER_MM))

    # Quotient & Remainder (integer floor division) of 前進Steps by 上升時間(秒)
    speed_parameter = int(advance_steps // float(rise_time_s))

    if speed_parameter <= 0:
        raise ParameterValidationError(
            f"Calculated 速度參數 is {speed_parameter} steps/s. "
            "Increase 前進距離(mm) or decrease 上升時間(秒)."
        )
    if speed_parameter > config.MAX_SAFE_SPEED_STEPS_S:
        raise ParameterValidationError(
            f"Calculated 速度參數 ({speed_parameter} steps/s) exceeds safe motor "
            f"limit ({config.MAX_SAFE_SPEED_STEPS_S} steps/s)."
        )

    repeat_cycles_int = int(repeat_cycles)

    # --- 3. Write to Global Variable Store (2015myGlobal 1.vi equivalent) ---
    state.update_s1_globals(
        advance_distance_mm=float(advance_distance_mm),
        rise_time_s=float(rise_time_s),
        dwell_time_s=float(dwell_time_s),
        repeat_cycles=repeat_cycles_int,
        advance_steps=advance_steps,
        speed_parameter=speed_parameter,
    )

    return {
        "前進距離(mm)": state.advance_distance_mm,
        "上升時間(秒)": state.rise_time_s,
        "停留時間(秒)": state.dwell_time_s,
        "重複次數": state.repeat_cycles,
        "前進STEP": state.advance_steps,
        "速度參數": state.speed_parameter,
        "total_commands_S3": state.repeat_cycles * 2,
    }


if __name__ == "__main__":
    print("==========================================================")
    print("  STAGE 1 (S1): Parameter Setting Verification Test")
    print("  Reference: S1_parametetsSetting2017.vi")
    print("==========================================================\n")

    global_state = GlobalSystemState()

    # Prompt user with LabVIEW defaults from Image 7 (4 mm, 4 s, 4 s, 5 cycles)
    try:
        raw_dist = input(f"Enter 前進距離(mm) [default {config.DEFAULT_ADVANCE_MM}]: ").strip()
        dist_mm = float(raw_dist) if raw_dist else config.DEFAULT_ADVANCE_MM

        raw_rise = input(f"Enter 上升時間(秒) [default {config.DEFAULT_RISE_TIME_S}]: ").strip()
        rise_s = float(raw_rise) if raw_rise else config.DEFAULT_RISE_TIME_S

        raw_dwell = input(f"Enter 停留時間(秒) [default {config.DEFAULT_DWELL_TIME_S}]: ").strip()
        dwell_s = float(raw_dwell) if raw_dwell else config.DEFAULT_DWELL_TIME_S

        raw_cycles = input(f"Enter 重複次數     [default {config.DEFAULT_REPEAT_CYCLES}]: ").strip()
        cycles = int(raw_cycles) if raw_cycles else config.DEFAULT_REPEAT_CYCLES

        result = calculate_and_set_s1_parameters(
            state=global_state,
            advance_distance_mm=dist_mm,
            rise_time_s=rise_s,
            dwell_time_s=dwell_s,
            repeat_cycles=cycles,
        )

        print("\n--- [Set UP] Global Variables Written (2015myGlobal 1.vi) ---")
        print(f"  前進距離 (mm)      : {result['前進距離(mm)']:.2f} mm")
        print(f"  上升時間 (秒)      : {result['上升時間(秒)']:.2f} s")
        print(f"  停留時間 (秒)      : {result['停留時間(秒)']:.2f} s")
        print(f"  重複次數           : {result['重複次數']} cycles ({result['total_commands_S3']} total S3 moves)")
        print("  -----------------------------------------------------------")
        print(f"  -> 前進STEP (I32)  : {result['前進STEP']} steps")
        print(f"  -> 速度參數 (I32)  : {result['速度參數']} steps/s")
        print(f"  -> S2 Command Preview: @0SI{result['速度參數']},{result['速度參數']},{result['速度參數']}\\r\\n")
        print("-------------------------------------------------------------\n")
        print("[SUCCESS] S1 Parameter Setting verified and locked in Global State.")

    except Exception as exc:
        print(f"\n[FAILURE] S1 Validation Error: {exc}")