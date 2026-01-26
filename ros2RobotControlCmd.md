# ROS2 Package Creation & Debugging Log (robot_keyboard)

This document traces the terminal commands and troubleshooting steps for creating a ROS2 Python package for robot keyboard control.

## 1. Package Creation

Created a new ROS2 package named `robot_keyboard` using the `ament_python` build type.

```bash
mkdir -p ~/rc/src
cd ~/rc/src
ros2 pkg create robot_keyboard --build-type ament_python
```

### Folder Structure Created

* `package.xml`
* `setup.py`
* `setup.cfg`
* `robot_keyboard/` (source folder)
* `resource/`
* `test/`

## 2. Development Steps

1. **Source Code**: Created `keyboard_control.py` inside the `robot_keyboard/robot_keyboard/` directory.
2. **Setup**: Modified `setup.py` to register the `keyboard_control` entry point.
3. **Environment**: Created and activated a Python virtual environment (`venv`).
4. **Build**: Used `colcon` to build the package.

```bash
cd ~/rc
colcon build --symlink-install
source install/setup.bash
```

## 3. Debugging & Error Resolution

### Issue 1: IndentationError

**Error:** `IndentationError: expected an indented block after class definition`

* **Cause**: The `__init__` method was not correctly indented within the class.
* **Fix**: Corrected indentation in `keyboard_control.py`.

### Issue 2: TimeoutError

**Error:** `TimeoutError: [Errno 110] Connection timed out`

* **Cause**: The script could not connect to the ESP8266 robot IP/Port.
* **Fix**: Ensured the robot was ON and the laptop was connected to the correct WiFi (`Robot_AP`).

### Issue 3: AttributeError

**Error:** `AttributeError: 'KeyboardControl' object has no attribute 'send_cmd'`

* **Cause**: The `send_cmd` method was missing or named differently in the `KeyboardControl` class.
* **Fix**: Added the `send_cmd` method to handle sending commands over the socket.

## 4. Final Verification

Successfully connected to the ESP8266 robot and initiated the keyboard control node.

```text
[INFO] [ ... ] [keyboard_control]: Connected to ESP8266 robot
```

---
> [!NOTE]
> This log demonstrates the typical iterative process of developing and debugging a ROS2 node interfacing with external hardware.
