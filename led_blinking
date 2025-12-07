# 🤖 NodeMCU Serial LED Controller

This project provides an Arduino sketch for the **ESP8266 (NodeMCU)** that allows control of the built-in LED via serial commands. The sketch handles simple commands like `ON`, `OFF`, and customizable `BLINK`.

The built-in LED on most NodeMCU boards is connected to **GPIO2 (D4)** and is **active LOW** (meaning sending a `LOW` signal turns the LED **ON**).

---

## 💻 Arduino Sketch: `nodemcu_serial_led.ino`

```cpp
// nodemcu_serial_led.ino
// NodeMCU (ESP8266) serial LED controller — built-in LED is active LOW (GPIO2)

const int LED = LED_BUILTIN; // usually GPIO2 (D4)
unsigned long blink_period = 500UL; // ms
bool blinking = false;
unsigned long last_toggle = 0;
bool led_state = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // OFF (active LOW)
  Serial.println("Ready");
}

void loop() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toUpperCase();

    if (line == "ON") {
      blinking = false;
      digitalWrite(LED, LOW); // ON
      Serial.println("OK:ON");

    } else if (line == "OFF") {
      blinking = false;
      digitalWrite(LED, HIGH); // OFF
      Serial.println("OK:OFF");

    } else if (line.startsWith("BLINK")) {
      unsigned long requested = 0UL;
      int sp = line.indexOf(' ');
      if (sp > 0) requested = (unsigned long) line.substring(sp + 1).toInt();
      else requested = 500UL;
      if (requested < 50UL) requested = 50UL;
      blink_period = requested;
      blinking = true;
      last_toggle = millis();
      led_state = false;
      Serial.printf("OK:BLINK %lu\n", blink_period);

    } else if (line == "STOP") {
      blinking = false;
      Serial.println("OK:STOP");

    } else {
      Serial.println("ERR:UNKNOWN");
    }
  }

  if (blinking) {
    unsigned long now = millis();
    // Toggles the LED state every half of the blink period
    if (now - last_toggle >= blink_period / 2) { 
      last_toggle = now;
      led_state = !led_state;
      digitalWrite(LED, led_state ? LOW : HIGH); // LOW = ON, HIGH = OFF
    }
  }
}
