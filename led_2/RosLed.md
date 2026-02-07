# ESP8266 Built-in LED Blink using ROS 2 Jazzy (Ubuntu 24.04 on WSL)

## PART 1: WINDOWS (WSL USB SETUP)

1. **Check WSL version:**

   ```powershell
   wsl --version
   ```

2. **Install USB attach tool:**

   ```powershell
   winget install usbipd
   ```

   *Note: Reboot Windows after installation.*

3. **Plug ESP8266 and list USB devices:**

   ```powershell
   usbipd list
   ```

   Example output:
   `1-9 1A86:7523 USB-SERIAL CH340`

4. **Bind device (ONE TIME ONLY):**

   ```powershell
   usbipd bind --busid <BUSID>
   ```

5. **Attach device to WSL (EVERY BOOT / REPLUG):**

   ```powershell
   usbipd attach --busid <BUSID> --wsl
   ```

## PART 2: UBUNTU (WSL SIDE)

1. **Confirm device is visible:**

   ```bash
   ls /dev/ttyUSB*
   # Expected: /dev/ttyUSB0
   ```

2. **Fix permission:**

   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

## PART 3: ARDUINO IDE + ESP8266

### 🛠️ Install Arduino IDE on Ubuntu (AppImage Method)

1. **Update Ubuntu:**

   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

2. **Install dependencies:**

   ```bash
   sudo apt install -y wget libgtk-3-0 libxss1 libasound2
   ```

3. **Download Arduino IDE (official Linux build):**

   ```bash
   cd ~
   wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.2_Linux_64bit.AppImage
   ```

4. **Make it executable:**

   ```bash
   chmod +x arduino-ide_2.3.2_Linux_64bit.AppImage
   ```

5. **Run Arduino IDE:**

   ```bash
   ./arduino-ide_2.3.2_Linux_64bit.AppImage
   ```

   ✨ The GUI Arduino IDE window will open.

### ⚙️ Configure ESP8266 Board Support

1. **Add ESP8266 URL:**
   Arduino IDE → File → Preferences → Additional Boards Manager URLs:
   `http://arduino.esp8266.com/stable/package_esp8266com_index.json`

2. **Install ESP8266 boards:**
   Tools → Board → Boards Manager → Search `esp8266` → Install.

3. **Select board & port:**
   - Tools → Board → NodeMCU 1.0 (ESP-12E)
   - Tools → Port → /dev/ttyUSB0

### 📄 Upload ESP8266 Code

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

✔ Test using Serial Monitor (1 → LED ON, 0 → LED OFF).

## PART 4: ROS 2 WORKSPACE SETUP

1. **Create workspace:**

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Create ROS package:**

   ```bash
   ros2 pkg create led_control --build-type ament_python
   ```

## PART 5: PYTHON VIRTUAL ENV (Ubuntu 24.04 SAFE WAY)

1. **Create venv:**

   ```bash
   cd ~/ros2_ws/src/led_control
   python3 -m venv venv
   ```

2. **Activate venv:**

   ```bash
   source venv/bin/activate
   ```

3. **Install pyserial:**

   ```bash
   pip install pyserial
   ```

## PART 6: ROS NODE CODE

1. **Create ROS Python node:**
   Save as `~/ros2_ws/src/led_control/led_control/led_serial_node.py`:

   ```python
   import rclpy
   from rclpy.node import Node
   import serial
   import time

   class LedNode(Node):
       def __init__(self):
           super().__init__('led_node')
           self.ser = serial.Serial('/dev/ttyUSB0', 9600)
           time.sleep(2)
           self.state = False
           self.timer = self.create_timer(1.0, self.toggle)
           self.get_logger().info("LED control started")

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
   ```

2. **Update `setup.py`:**
   Add to `entry_points`:

   ```python
   entry_points={
       'console_scripts': [
           'led_serial = led_control.led_serial_node:main',
       ],
   },
   ```

## PART 7: BUILD & RUN ROS NODE

1. **Build workspace:**

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

2. **Source environment:**

   ```bash
   source install/setup.bash
   source ~/ros2_ws/src/led_control/venv/bin/activate
   ```

3. **Run ROS node:**

   ```bash
   ros2 run led_control led_serial
   ```

## ERRORS & SOLUTIONS

- **/dev/ttyUSB0 not visible:** (WSL) Run `usbipd attach` on Windows.
- **usbipd: Device is not shared:** Run `usbipd bind` first.
- **externally-managed-environment:** Use a virtual environment (Part 5).
- **No executable found:** Ensure `colcon build` was successful and sourced correctly.

---
> [!IMPORTANT]
> Always ensure the device is attached to WSL using `usbipd attach` if you replug the ESP8266.
