# ESP8266 Custom LED Blink with ROS2

This guide covers how to control an LED on an ESP8266 using ROS2.

## ESP8266 Arduino Code

```cpp
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "Robot_AP";
const char* password = "robot123";

ESP8266WebServer server(80);

const int LED_PIN = LED_BUILTIN; // Usually D4 on NodeMCU

void handleRoot() {
  server.send(200, "text/plain", "ESP8266 LED Control Server");
}

void handleLedOn() {
  digitalWrite(LED_PIN, LOW); // LED_BUILTIN is active LOW
  server.send(200, "text/plain", "LED IS ON");
}

void handleLedOff() {
  digitalWrite(LED_PIN, HIGH);
  server.send(200, "text/plain", "LED IS OFF");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Turn off initially

  WiFi.softAP(ssid, password);
  Serial.println("AP Mode Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/on", handleLedOn);
  server.on("/off", handleLedOff);
  server.begin();
}

void loop() {
  server.handleClient();
}
```

## ROS2 LED Control Node (Python)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

ESP_IP = "192.168.4.1"

class LedControlNode(Node):
    def __init__(self):
        super().__init__('led_control_node')
        self.subscription = self.create_subscription(
            String,
            'led_command',
            self.listener_callback,
            10)
        self.get_logger().info("LED Control Node Started")

    def listener_callback(self, msg):
        command = msg.data.lower()
        try:
            if command == "on":
                requests.get(f"http://{ESP_IP}/on", timeout=0.5)
                self.get_logger().info("LED Turned ON")
            elif command == "off":
                requests.get(f"http://{ESP_IP}/off", timeout=0.5)
                self.get_logger().info("LED Turned OFF")
        except Exception as e:
            self.get_logger().error(f"Failed to reach ESP: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LedControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Usage

1. **Connect** to the Wi-Fi `Robot_AP` (password `robot123`).
2. **Run** the ROS2 node:

   ```bash
   ros2 run my_pkg led_control_node
   ```

3. **Publish** commands:

   ```bash
   ros2 topic pub /led_command std_msgs/msg/String "{data: 'on'}"
   ros2 topic pub /led_command std_msgs/msg/String "{data: 'off'}"
   ```
