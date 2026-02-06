# ESP8266 Serial LED Control with ROS2 Jazzy

This guide explains how to control an ESP8266 built-in LED via Serial communication using ROS2 on WSL2.

## 1. Windows USB Setup (WSL)

Since WSL handles USB devices differently, you need `usbipd`.

1. **Install usbipd on Windows:**

   ```powershell
   winget install usbipd
   ```

2. **List USB devices:**

   ```powershell
   usbipd list
   ```

3. **Bind and Attach your ESP8266:**

   ```powershell
   usbipd bind --busid <BUSID>
   usbipd attach --busid <BUSID> --wsl
   ```

## 2. Ubuntu Setup (WSL side)

1. **Confirm device:**

   ```bash
   ls /dev/ttyUSB*
   # Expect: /dev/ttyUSB0
   ```

2. **Permissions:**

   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

## 3. ESP8266 Arduino Code

Upload this code to your ESP8266 via Arduino IDE.

```cpp
#define LED_PIN LED_BUILTIN

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Off initially (active LOW)
    Serial.begin(9600);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '1') digitalWrite(LED_PIN, LOW);   // ON
        if (c == '0') digitalWrite(LED_PIN, HIGH);  // OFF
    }
}
```

## 4. ROS2 Workspace & Python Node

### Create Package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create led_control --build-type ament_python
```

### Install Dependencies (Python Venv)

Ubuntu 24.04 requires a virtual environment for `pip`:

```bash
cd ~/ros2_ws/src/led_control
python3 -m venv venv
source venv/bin/activate
pip install pyserial
```

### ROS Node (`led_serial_node.py`)

Save in `~/ros2_ws/src/led_control/led_control/`:

```python
import rclpy
from rclpy.node import Node
import serial
import time

class LedNode(Node):
    def __init__(self):
        super().__init__('led_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        time.sleep(2) # Wait for serial to initialize
        self.state = False
        self.timer = self.create_timer(1.0, self.toggle)
        self.get_logger().info("Serial LED control started")

    def toggle(self):
        self.ser.write(b'1' if not self.state else b'0')
        self.state = not self.state
        self.get_logger().info(f"LED State: {'ON' if self.state else 'OFF'}")

def main():
    rclpy.init()
    node = LedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Update `setup.py`

Add to `entry_points`:

```python
entry_points={
    'console_scripts': [
        'led_serial = led_control.led_serial_node:main',
    ],
},
```

## 5. Build and Run

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
source ~/ros2_ws/src/led_control/venv/bin/activate # Re-activate venv
ros2 run led_control led_serial
```

---
> [!IMPORTANT]
> Always ensure the device is attached to WSL using `usbipd attach` if you replug the ESP8266.
