# e-puck Robot Control (Webots + Python)

This document contains Python controller code for the e-puck robot in Webots, featuring obstacle avoidance and wall-following logic.

## 1. Obstacle Avoidance Controller

This script uses infrared distance sensors to avoid obstacles while maintaining forward movement.

```python
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Set motors to velocity mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Initial speed
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Distance sensors
sensor_names = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
sensors = []
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

MAX_SPEED = 6.28
TURN_SPEED = 3.0
THRESHOLD = 80

while robot.step(timestep) != -1:
    values = [s.getValue() for s in sensors]
    
    # ps7 is Front-Left, ps0 is Front-Right
    # ps6 and ps1 are angled slightly, good for corners
        
    # Check the right side of the front
    right_obstacle = values[0] > THRESHOLD or values[1] > THRESHOLD
        
    # Check the left side of the front
    left_obstacle = values[7] > THRESHOLD or values[6] > THRESHOLD
    
    if left_obstacle:
        # Obstacle on the left? Turn Right.
        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(-TURN_SPEED)
            
    elif right_obstacle:
        # Obstacle on the right? Turn Left.
        left_motor.setVelocity(-TURN_SPEED)
        right_motor.setVelocity(TURN_SPEED)
            
    else:
        # No obstacle? Go Forward
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)
```

## 2. Wall Following Controller

This script uses a PID-like logic (proportional control) to keep a specific distance from a wall on the left.

```python
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# --- TUNING PARAMETERS ---
MAX_SPEED = 10.28
CRUISE_SPEED = 4.0
TARGET_WALL_DIST = 150.0  # Distance to keep from wall
KP = 0.02                 # Proportional gain
DEADZONE = 5.0            # Error deadzone

sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
sensors = []
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

while robot.step(timestep) != -1:
    values = [s.getValue() for s in sensors]
    
    # Front sensors (to detect walls in front)
    front_wall = values[0] > 80 or values[7] > 80
        
    # Left sensor (to follow the wall)
    left_sensor_value = values[5]
    
    if front_wall:
        # Sharp turn right if wall is in front
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(-2.0)
    else:
        # WALL FOLLOWING LOGIC
        error = TARGET_WALL_DIST - left_sensor_value
                
        # 1. Apply Deadzone
        if abs(error) < DEADZONE:
            correction = 0
        else:
            # 2. Apply a small correction
            correction = error * KP
            
        # Calculate speeds
        v_left = CRUISE_SPEED - correction
        v_right = CRUISE_SPEED + correction
        
        # Clamp speeds to limits
        v_left = max(min(v_left, MAX_SPEED), -MAX_SPEED)
        v_right = max(min(v_right, MAX_SPEED), -MAX_SPEED)
        
        left_motor.setVelocity(v_left)
        right_motor.setVelocity(v_right)
```

---
> [!TIP]
> Use the `KP` parameter to tune how aggressively the robot corrects its distance from the wall.
