"""
config.py
Hardware constants, LabVIEW kinematic constants, and safety limits.
"""

# --- Serial Motor Hardware Configuration (from S2 & S3 Front Panels) ---
DEFAULT_COM_PORT = "COM4"
DEFAULT_BAUD_RATE = 9600
SERIAL_TIMEOUT_S = 1.0
POS_READOUT_WAIT_S = 0.100      # 100 ms wait inside POSiReadout.vi
POS_READOUT_BYTES = 10          # 10 bytes read in POSiReadout.vi
INITIAL_HOME_STEPS = 1000       # Initial @0X1000 move in S3Motor_control2017m.vi

# --- NI-DAQmx & Load Cell Configuration (Verified on Hardware) ---
DEFAULT_DAQ_CHANNEL = "Dev1/ai1"
DAQ_MIN_VOLTAGE = -10.0
DAQ_MAX_VOLTAGE = 10.0
DAQ_SAMPLE_RATE_HZ = 1000.0
DAQ_BLOCK_SIZE = 500            # 500-sample block average for static/stepped reads
DAQ_CONTINUOUS_BUFFER = 500000  # 500k buffer to prevent Status Code -200279

# --- Verified Calibration & Mechanical Constants ---
STEPS_PER_MM = 1600.0           # Exact multiplier in S1_parametetsSetting2017.vi
CALIBRATED_GRAMS_PER_VOLT = 505.97  # Verified via 500g Reverse-Tare calibration

# --- Default S1 Front Panel Parameters (from S1_parametetsSetting2017.vi) ---
DEFAULT_ADVANCE_MM = 4.0        # 前進距離(mm)
DEFAULT_RISE_TIME_S = 4.0       # 上升時間(秒)
DEFAULT_DWELL_TIME_S = 4.0      # 停留時間(秒)
DEFAULT_REPEAT_CYCLES = 5       # 重複次數

# --- Software Safety Limits (Update if your hardware requires tighter bounds) ---
MAX_SAFE_ADVANCE_MM = 15.0      # Max allowed indentation depth (mm) -> 24,000 steps
MAX_SAFE_SPEED_STEPS_S = 10000  # Max allowed stepper speed (steps/s)
MAX_SAFE_FORCE_GRAMS = 1000.0   # Emergency cutoff force (grams)

# --- Software Safety Cutoffs ---
MIN_SAFE_STEPS = 0                  # Minimum allowed absolute motor position (steps)
MAX_SAFE_STEPS = 16000              # Maximum allowed absolute motor position (10.0 mm)
MAX_SAFE_ADVANCE_MM = 9.0           # Max allowed advance from 1000-step baseline (14,400 steps)
MAX_SAFE_SPEED_STEPS_S = 6400       # Max allowed stepper speed (4.0 mm/s)
MAX_SAFE_FORCE_GRAMS = 1500.0       # Default test force cutoff (grams)
ABSOLUTE_MAX_FORCE_GRAMS = 4000.0   # Hard 80% ceiling for 5kg (10V) load cell