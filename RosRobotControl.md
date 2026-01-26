# ROS2 Robot Control (ESP8266 + Keyboard Joystick)

A complete guide to building a ROS2-controlled robot using an ESP8266 in AP mode and keyboard input.

## ESP8266 Arduino Code (HTTP + UDP)

This code supports both HTTP (for testing/browser) and UDP (for low-latency control) connections.

```cpp
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>

const char* ssid = "Robot_AP";
const char* password = "robot123";

ESP8266WebServer server(80);
WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[255];

// Motor Pins (NodeMCU)
#define M11 D1
#define M12 D2
#define M21 D3
#define M22 D4

void setup() {
    Serial.begin(115200);
    pinMode(M11, OUTPUT);
    pinMode(M12, OUTPUT);
    pinMode(M21, OUTPUT);
    pinMode(M22, OUTPUT);
    
    stopRobot();

    WiFi.softAP(ssid, password);
    Serial.println("ESP8266 AP MODE STARTED");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", []() {
        server.send(200, "text/html", "<h1>Robot Control</h1><button onclick=\"fetch('/fwd')\">FWD</button>...");
    });
    server.on("/fwd", fwd);
    server.on("/rev", rev);
    server.on("/left", left);
    server.on("/right", right);
    server.on("/stop", stopRobot);

    server.begin();
    Udp.begin(localUdpPort);
}

void fwd() {
    digitalWrite(M11, HIGH); digitalWrite(M12, LOW);
    digitalWrite(M21, HIGH); digitalWrite(M22, LOW);
    if (server.active()) server.send(200, "text/plain", "Forward");
}

void rev() {
    digitalWrite(M11, LOW); digitalWrite(M12, HIGH);
    digitalWrite(M21, LOW); digitalWrite(M22, HIGH);
    if (server.active()) server.send(200, "text/plain", "Reverse");
}

void left() {
    digitalWrite(M11, HIGH); digitalWrite(M12, LOW);
    digitalWrite(M21, LOW); digitalWrite(M22, HIGH);
    if (server.active()) server.send(200, "text/plain", "Left");
}

void right() {
    digitalWrite(M11, LOW); digitalWrite(M12, HIGH);
    digitalWrite(M21, HIGH); digitalWrite(M22, LOW);
    if (server.active()) server.send(200, "text/plain", "Right");
}

void stopRobot() {
    digitalWrite(M11, LOW); digitalWrite(M12, LOW);
    digitalWrite(M21, LOW); digitalWrite(M22, LOW);
    if (server.active()) server.send(200, "text/plain", "Stopped");
}

void loop() {
    server.handleClient();
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        int len = Udp.read(incomingPacket, 255);
        if (len > 0) incomingPacket[len] = 0;
        String cmd = String(incomingPacket);
        if (cmd == "fwd") fwd();
        else if (cmd == "rev") rev();
        else if (cmd == "left") left();
        else if (cmd == "right") right();
        else if (cmd == "stop") stopRobot();
    }
}
```

## ROS2 Python Bridge (`cmdvel_to_esp.py`)

This node converts `/cmd_vel` messages into HTTP commands for the ESP8266.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests

ESP_IP = "192.168.4.1"

class CmdVelToESP(Node):
    def __init__(self):
        super().__init__('cmdvel_to_esp')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvel_callback,
            10
        )
        self.last_cmd = "stop"
        self.get_logger().info("Keyboard joystick -> ESP bridge started")

    def cmdvel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if linear > 0.1: cmd = "fwd"
        elif linear < -0.1: cmd = "rev"
        elif angular > 0.1: cmd = "left"
        elif angular < -0.1: cmd = "right"
        else: cmd = "stop"

        if cmd != self.last_cmd:
            self.send_cmd(cmd)
            self.last_cmd = cmd

    def send_cmd(self, cmd):
        try:
            url = f"http://{ESP_IP}/{cmd}"
            requests.get(url, timeout=0.5)
            self.get_logger().info(f"Sent command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"ESP not reachable: {e}")

def main():
    rclpy.init()
    node = CmdVelToESP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step-by-Step Setup

### Phase 1: Hardware & ESP8266

1. **Upload** the Arduino code above to your NodeMCU.
2. **Connect** your laptop to the Wi-Fi network: `Robot_AP` (Password: `robot123`).
3. **Test** by visiting `http://192.168.4.1` in your browser. Click the buttons to ensure the robot moves.

### Phase 2: ROS2 Workspace Setup

1. **Create Package**:

   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create robot_teleop_bridge --build-type ament_python --dependencies rclpy geometry_msgs
   ```

2. **Add Code**: Save the Python code as `cmdvel_to_esp.py` in `~/ros2_ws/src/robot_teleop_bridge/robot_teleop_bridge/`.
3. **Make Executable**: `chmod +x cmdvel_to_esp.py`
4. **Register Node**: Edit `setup.py` and add to `entry_points`:

   ```python
   'console_scripts': [
       'cmdvel_to_esp = robot_teleop_bridge.cmdvel_to_esp:main',
   ],
   ```

5. **Build**:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

### Phase 3: Running (3 Terminals)

1. **Terminal 1 (Bridge)**:

   ```bash
   ros2 run robot_teleop_bridge cmdvel_to_esp
   ```

2. **Terminal 2 (Keyboard)**:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **Terminal 3 (Monitor)**:

   ```bash
   ros2 topic echo /cmd_vel
   ```

## Controls

- **w**: Forward
- **a**: Left
- **d**: Right
- **s**: Reverse
- **space**: Stop
