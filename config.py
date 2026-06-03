# config.py

# Spool
SPOOL_DIAMETER = 0.045  # m

# Gear ratios
GEAR_RATIO_SLOW = (36/100) * (45/80)  # switch activé (avion en bas)
GEAR_RATIO_FAST = 0.2                # switch non activé (vol libre)

# Motor limits
MAX_RPM_SLOW = 500
MAX_RPM_FAST = 3000
CURRENT_LIM = 60.0        # A continu
CURRENT_LIM_PEAK = 60.0   # A peak
TORQUE_LIM = 4.0        # Nm

# Trajectory — normal mode
ACCEL_LIMIT = 50.0        # tours/s²
DECEL_LIMIT = 50.0        # tours/s²

# Trajectory — fast mode
ACCEL_LIMIT_FAST = 50.0  # tours/s²
DECEL_LIMIT_FAST = 50.0   # tours/s²

# Velocity ramp
VEL_RAMP_RATE = 50.0      # tours/s²

# Tracking
TRACKING_FREQ = 10       # Hz control loop

# Hall sensor
HALL_PIN = 6
N_MAGNETS = 1
BOUNCE_TIME = 0.001      # s

# Switch
SWITCH_PIN = 18

# Servo
SERVO_PIN = 16

# Socket
HOST = '0.0.0.0'
PORT = 5005

# Control loop
CONTROL_FREQ = 10        # Hz