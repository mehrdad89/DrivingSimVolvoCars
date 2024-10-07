# constants.py

# Vehicle Parameters
MAX_ACCELERATION = 1.5  # m/s²
MAX_DECELERATION = 2.0  # m/s²
JERK_LIMIT = 0.5  # m/s³

# Weather Conditions
DEFAULT_WEATHER_TYPE = 'clear'
RAIN_SPEED_REDUCTION = 0.85
SNOW_SPEED_REDUCTION = 0.75

# Road Parameters
DEFAULT_CURVATURE = 0.05
TARGET_HEADWAY_TIME = 2.0  # seconds
EMERGENCY_BRAKE_ACCELERATION = 5.0  # m/s²

# Time intervals
STEP_INTERVAL = 0.01  # seconds
SIMULATION_DURATION = 30.0  # seconds