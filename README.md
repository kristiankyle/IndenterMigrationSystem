# Indenter Migration Project

## Overview

The **Indenter Migration Project** is a modernization effort to migrate an existing biomechanical indenter system from **LabVIEW to Python**.

The original indenter system was developed in LabVIEW and is used to control the indenter mechanism, communicate with the data acquisition hardware, collect load-cell measurements, and perform force-related calculations.

The goal of this project is to reproduce and improve the existing system using Python, making the application **more flexible, maintainable, cost-effective, and easier to develop and extend**.

The migration focuses on preserving the functionality of the original LabVIEW system while gradually replacing its components with Python-based implementations.

---

## Project Goals

The main objectives of the migration are:

- Migrate the existing LabVIEW indenter application to Python.
- Maintain the functionality and behavior of the original system.
- Improve the flexibility and maintainability of the software.
- Reduce dependency on proprietary software and licensing costs.
- Make the system easier to modify and extend.
- Improve integration with Python-based data processing and research workflows.
- Provide a foundation for future automation and analysis.
- Ensure accurate force measurement and motor control.

---

## Original System

The existing system was developed using **LabVIEW** and consists of several hardware and software components.

### Main Components

- Indenter mechanism
- Stepper motor
- NEMA 17 motor
- Load cell
- NI USB-6218 Data Acquisition Device
- NI-DAQmx
- LabVIEW control and measurement logic
- Windows-based measurement environment

The LabVIEW application handles the communication between the computer, data acquisition hardware, load cell, and motor control system.

---

## New System

The new implementation uses **Python** as the primary programming environment.

### Technology Stack

| Component            | Technology              |
| -------------------- | ----------------------- |
| Programming Language | Python                  |
| Motor Control        | Stepper motor / NEMA 17 |
| Data Acquisition     | NI USB-6218             |
| DAQ Communication    | NI-DAQmx                |
| Python DAQ Interface | `nidaqmx`               |
| Sensor               | Load Cell               |
| Operating System     | Windows                 |
| Original System      | LabVIEW                 |

---

## System Architecture

The basic data and control flow is:

```text
                    ┌──────────────────┐
                    │   Python App     │
                    │                  │
                    │ Motor Control    │
                    │ Data Acquisition │
                    │ Calculations     │
                    └────────┬─────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
              ▼                             ▼
     ┌─────────────────┐           ┌─────────────────┐
     │ Stepper Motor   │           │ NI USB-6218     │
     │    NEMA 17      │           │ Data Acquisition│
     └─────────────────┘           └────────┬────────┘
                                            │
                                            ▼
                                     ┌──────────────┐
                                     │   Load Cell  │
                                     └──────────────┘
                                            │
                                            ▼
                                     Force Measurement
```

The Python application controls the indenter movement while simultaneously acquiring measurement data from the load cell through the NI data acquisition device.

---

## Migration Strategy

The migration is being performed incrementally rather than replacing the entire LabVIEW application at once.

Each major component of the original system is being identified, tested, and reproduced in Python.

### Migration Flow

```text
Original LabVIEW System
          │
          ▼
Identify Existing Functions
          │
          ▼
Understand LabVIEW Logic
          │
          ▼
Reproduce Function in Python
          │
          ▼
Test Against Original Behavior
          │
          ▼
Validate Hardware
          │
          ▼
Integrate Components
          │
          ▼
Python-Based Indenter System
```

This approach allows individual components to be tested before they are integrated into the complete system.

---

## Current Development Areas

### 1. Kinematics

The first stage focuses on understanding and reproducing the indenter's movement and positioning logic.

The Python implementation reproduces the kinematic calculations used by the original LabVIEW system.

Current work includes:

- Motor movement calculations
- Position calculations
- Step calculations
- Movement direction
- Indenter positioning
- Validation of calculated movement

---

### 2. Stepper Motor Control

The system uses a **NEMA 17 stepper motor** to control the movement of the indenter.

The Python implementation is being developed to reproduce the motor-control behavior of the original LabVIEW system.

The motor-control layer is responsible for:

- Motor movement
- Direction control
- Step generation
- Positioning
- Movement sequencing

---

### 3. Load Cell Data Acquisition

The load cell provides the physical force measurement at the indenter tip.

The measurement chain is:

```text
Load Cell
    │
    ▼
NI USB-6218
    │
    ▼
NI-DAQmx
    │
    ▼
Python nidaqmx
    │
    ▼
Force Data
```

The Python application communicates with the NI USB-6218 through the NI-DAQmx driver and the Python `nidaqmx` interface.

---

## Force Calibration

The load-cell output must be converted from its electrical measurement into a physical force value.

The calibration process establishes the relationship between the measured voltage and the applied force.

A linear calibration model is used:

```text
Force = Voltage × Calibration Coefficient + Zero-Point Offset
```

or:

```text
y = mx + b
```

Where:

- `x` = measured voltage
- `m` = calibration coefficient
- `b` = zero-point offset
- `y` = calculated force

The calibration process is important to ensure that the force reported by the Python system accurately represents the physical force applied to the indenter.

---

## Development Stages

The migration can be divided into several stages.

### S1 — Kinematics

- Understand the original LabVIEW kinematic logic.
- Reproduce calculations in Python.
- Verify expected movement and positioning.

### S2 — Mathematical Model

- Reproduce mathematical calculations.
- Validate formulas and parameters.
- Compare Python results with LabVIEW results.

### S3 — Motor Control

- Connect the NEMA 17 stepper motor.
- Implement movement control.
- Verify direction and positioning.
- Compare behavior with the LabVIEW system.

### S4 — Data Acquisition

- Establish communication with the NI USB-6218.
- Configure NI-DAQmx.
- Acquire load-cell signals.
- Read measurements through Python.

### S5 — Load-Cell Calibration

- Collect calibration measurements.
- Determine calibration coefficient.
- Determine zero-point offset.
- Convert voltage measurements into force.

### S6 — System Integration

Combine:

```text
Motor Control
      +
Kinematics
      +
Data Acquisition
      +
Load Cell
      +
Calibration
      +
Measurement Logic
```

into a single Python-based indenter application.

### S7 — Validation

The final Python implementation will be compared against the original LabVIEW system to verify:

- Movement accuracy
- Position accuracy
- Force measurement
- Calibration accuracy
- Data acquisition reliability
- Overall system behavior

---

## Important Hardware Considerations

The indenter system depends on the correct interaction between the software and physical hardware.

Changes to the NI-DAQmx environment can affect the existing LabVIEW system.

During development, compatibility between:

```text
LabVIEW
NI-DAQmx
NI USB-6218
Python
nidaqmx
Windows
```

must therefore be carefully managed.

The migration should avoid changing the hardware configuration unnecessarily while the Python implementation is being validated.

---

## Why Python?

Python provides several advantages for the future development of the system.

### Lower Software Cost

The migration can reduce dependence on proprietary development environments and associated licensing requirements.

### Easier Development

Python provides a large ecosystem of libraries for:

- Data acquisition
- Data processing
- Scientific computing
- Visualization
- Automation
- Machine learning
- Data analysis

### Better Integration

The system can eventually be integrated with Python-based research and analysis workflows without requiring data to move between separate programming environments.

### Maintainability

Python code can be organized into reusable modules, making it easier to modify individual parts of the system.

### Future Expansion

The Python implementation can provide a foundation for future capabilities such as:

- Automated experiments
- Real-time visualization
- Data logging
- Advanced force analysis
- Statistical analysis
- Machine learning
- Research automation

---

## Project Structure

The project is intended to be organized into modular components.

```text
indenter-migration/
│
├── README.md
│
├── src/
│   ├── motor/
│   │   └── motor_control.py
│   │
│   ├── kinematics/
│   │   └── kinematics.py
│   │
│   ├── daq/
│   │   └── daq_reader.py
│   │
│   ├── sensors/
│   │   └── load_cell.py
│   │
│   ├── calibration/
│   │   └── calibration.py
│   │
│   └── main.py
│
├── tests/
│   ├── test_kinematics.py
│   ├── test_calibration.py
│   └── test_daq.py
│
├── docs/
│   ├── labview_reference/
│   ├── hardware/
│   └── calibration/
│
└── requirements.txt
```

The exact structure may change as development progresses.

---

## Testing Philosophy

Each component should be tested independently before integration.

For example:

```text
Kinematics
    ↓
Unit Test
    ↓
Motor Control
    ↓
Hardware Test
    ↓
DAQ
    ↓
Sensor Test
    ↓
Calibration
    ↓
Integrated System
```

This helps identify whether an issue originates from the mathematical model, Python implementation, hardware, driver, or integration layer.

---

## Validation Against LabVIEW

The original LabVIEW system serves as the primary reference during migration.

Where possible, equivalent inputs should be provided to both systems and the resulting outputs compared.

```text
             Same Input
                 │
        ┌────────┴────────┐
        ▼                 ▼
    LabVIEW             Python
        │                 │
        ▼                 ▼
    Output A            Output B
        │                 │
        └────────┬────────┘
                 ▼
             Comparison
                 │
                 ▼
          Validation Result
```

The objective is not simply to create a Python program that works, but to demonstrate that the migrated system produces reliable and comparable results.

---

## Project Status

**Status: In Development**

Current focus:

- [x] Understand existing LabVIEW workflow
- [x] Reproduce initial kinematics calculations
- [x] Develop mathematical model
- [x] Connect/test NEMA 17 motor
- [x] Complete Python motor-control implementation
- [x] Implement complete NI-DAQmx acquisition
- [x] Implement load-cell acquisition
- [x] Complete calibration workflow
- [x] Integrate motor and DAQ systems
- [x] Validate Python results against LabVIEW
- [ ] Develop complete Python indenter application

---

## Expected Outcome

The final system will provide a Python-based replacement for the existing LabVIEW indenter application.

The completed system should allow researchers to:

1. Control the indenter.
2. Move the indenter according to the required kinematic parameters.
3. Acquire load-cell measurements.
4. Convert sensor measurements into force.
5. Record experimental data.
6. Perform measurements using a Python-based workflow.
7. Extend the system with additional analysis and automation in the future.

---

## Future Improvements

Potential future developments include:

- Real-time force/position graphs
- Automated indentation protocols
- Experiment configuration interface
- CSV/Excel data export
- Automatic calibration
- Experimental data logging
- Error handling and hardware safety checks
- Automated report generation
- Data visualization
- Integration with biomechanical analysis tools
- Machine-learning-based analysis

---

## Disclaimer

This project is a research and development migration of an existing laboratory system.

Hardware operation should be performed with appropriate supervision and safety precautions. Changes to motor control, force limits, DAQ configuration, or mechanical parameters should be validated before conducting actual experiments.

---

## License

This project is intended for research and laboratory development.

License information will be added as the project is finalized.
