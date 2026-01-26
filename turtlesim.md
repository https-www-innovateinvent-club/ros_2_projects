# ROS2 Turtlesim Tutorial (Jazzy)

A step-by-step guide to using Turtlesim, a great starting point for learning ROS2.

## 1. Prerequisites

Ensure ROS2 Jazzy is correctly installed and sourced:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

## 2. Installation

Install the turtlesim package:

```bash
sudo apt update
sudo apt install ros-jazzy-turtlesim -y
```

Verify installation:

```bash
ros2 pkg list | grep turtlesim
```

## 3. Running Turtlesim

**Terminal 1:** Start the turtlesim node.

```bash
ros2 run turtlesim turtlesim_node
```

A window with a blue background and a turtle should appear.

## 4. Keyboard Teleoperation

**Terminal 2:** Control the turtle using your keyboard.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtle_teleop_key
```

* **W/S**: Forward/Backward
* **A/D**: Rotate Left/Right

## 5. Identifying Nodes and Topics

Open a new terminal to inspect the system:

```bash
ros2 node list
# Output: /turtlesim, /teleop_turtle

ros2 topic list
# Key topic: /turtle1/cmd_vel
```

## 6. Manual Topic Publishing

Send a command directly to the turtle via a topic:

```bash
# Move Forward
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.0}}"

# Rotate
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 2.0}}"
```

## 7. Using Services

**Spawn a new turtle:**

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
```

**Reset or Clear:**

```bash
ros2 service call /clear std_srvs/srv/Empty
ros2 service call /reset std_srvs/srv/Empty
```

## 8. Automated Control with Python

Create a Python node to move the turtle in a curve.

**Create Package:**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create turtle_controller --build-type ament_python --dependencies rclpy geometry_msgs
```

**`turtle_controller/move_turtle.py`**:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_turtle)

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = TurtleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Register in `setup.py`**:

```python
entry_points={
    'console_scripts': [
        'move_turtle = turtle_controller.move_turtle:main',
    ],
},
```

**Build and Run:**

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run turtle_controller move_turtle
```

---
> [!NOTE]
> Turtlesim concepts map directly to real robots. `/cmd_vel` is the universal topic for robot movement commands.
