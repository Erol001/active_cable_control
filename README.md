# Active Cable Control WASP

Active cable control system for the Winged Aircraft Safety Platform (WASP). A Raspberry Pi runs the control server; a PC runs the GUI client over TCP.

## System Overview

```
PC (GUI) ──TCP/5005──► Raspberry Pi ──USB──► ODrive motor controller ──► Brushless motor ──► Spool
                                      ├──GPIO──► Hall sensor (spool RPM)
                                      ├──GPIO──► Switch sensor (aircraft at bottom / free flight)
                                      └──GPIO──► Servo (cable lock/unlock)
```

The spool winds/unwinds a cable whose length is controlled by targeting motor turns, with the gear ratio switching automatically based on the switch sensor state.

## Hardware

| Component | Role |
|-----------|------|
| ODrive | Brushless motor controller (position control, trap trajectory) |
| Hall sensor | Measures spool RPM and total cable deployed |
| Switch sensor | Detects aircraft state (landed vs. free flight) |
| Servo | Locks/unlocks the cable clamp |

## Software Architecture

| File | Description |
|------|-------------|
| `server.py` | Main server — runs on Raspberry Pi, manages all hardware and control loops |
| `odrive_motor.py` | ODrive interface (connect, enable, position/velocity/torque control) |
| `hall_sensor.py` | Hall effect sensor reader (RPM, total turns, direction) |
| `spool_tracker.py` | Cable length estimation from motor turns and gear ratio |
| `switch_sensor.py` | GPIO switch for flight mode detection |
| `servo_control.py` | Servo position control |
| `data_logger.py` | Telemetry logging to CSV |
| `config.py` | All hardware and control parameters |
| `gui_control.py` | Main Tkinter GUI (runs on PC) |
| `gui_exp1-4.py` | Experiment-specific GUIs |
| `gui_simple.py` | Minimal GUI for basic tests |

## Control Modes

- **Slow mode** — gear ratio `(36/100) × (45/80)`, max 500 RPM — used when aircraft is at the bottom
- **Fast mode** — gear ratio `0.2`, max 3000 RPM — used during free flight
- **Torque mode** — direct torque control with velocity limit
- **Tracking** — sinusoidal reference trajectory (configurable center, amplitude, frequency)
- **PRBS** — pseudo-random binary sequence input for system identification
- **Steps** — programmed step sequence for experiments

## Getting Started

### Raspberry Pi (server)

Install dependencies:
```bash
pip install odrive gpiozero
```

Start the server:
```bash
python server.py
```

The server listens on port `5005` and connects to the ODrive automatically at startup.

### PC (GUI client)

Install dependencies:
```bash
pip install matplotlib
```

Set the Raspberry Pi address in `gui_control.py` (`HOST = 'raspberrypi.local'`) then run:
```bash
python gui_control.py
```

## Configuration

All tunable parameters are in `config.py`:

```python
SPOOL_DIAMETER   = 0.045      # m
GEAR_RATIO_SLOW  = (36/100) * (45/80)
GEAR_RATIO_FAST  = 0.2
MAX_RPM_SLOW     = 500
MAX_RPM_FAST     = 3000
CURRENT_LIM      = 60.0       # A
TORQUE_LIM       = 4.0        # Nm
TORQUE_MAX_RPM   = 900        # RPM
ACCEL_LIMIT      = 50.0       # turns/s²
HALL_PIN         = 6
SWITCH_PIN       = 18
SERVO_PIN        = 16
PORT             = 5005
```

## TCP Protocol

Commands and telemetry are exchanged as newline-delimited JSON over a TCP socket.

**Commands (PC → Pi):**
```json
{"target_length": 2.5}
{"slow_mode": true}
{"custom_rpm": 800}
{"estop": true}
{"reset": true}
{"calibrate": true}
{"deploy": {"z": 3.0, "accel": 5.0, "decel": 3.0, "max_rpm": 1000}}
{"start_tracking": {"center": 2.0, "amplitude": 0.5, "freq": 0.5}}
{"start_prbs": {"center": 2.0, "amplitude": 0.5, "freq_min": 0.5, "freq_max": 2.0}}
{"servo_pos": 0.5}
{"servo_lock": true}
{"save_csv": true}
```

**Telemetry snapshot (Pi → PC, every control cycle):**
```json
{
  "t": 12.3,
  "motor_rpm": 245.0,
  "spool_rpm": 49.0,
  "cable_length": 1.85,
  "current": 3.2,
  "switch_active": false,
  "deploying": false,
  "tracking": true
}
```

## Calibration

The calibration routine spins the motor at `[100, 200, 400, 600, 800]` RPM, measures both motor and spool RPM, and saves the gear ratio measurements to a CSV file.
