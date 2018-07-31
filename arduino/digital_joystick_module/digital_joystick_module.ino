#include <base_arduino_module.h>

#define LED_PIN 6

BabblingModule module(BAB_MODULE_TYPE_DIGITAL_JOYSTICK);

volatile int last_report_time;

void setup() {
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);
  //while(!Serial);

  module.begin();

  digitalWrite(LED_PIN, HIGH);

  last_report_time = millis();
}

void loop() {
  uint8_t in_buf[BAB_MAX_PACKET_SIZE];
  int packet_sz = module.getPacket(in_buf);

  if (packet_sz > 0) {
    switch (in_buf[0]) {
      default:
        Serial.println("Unknown command");
        break;
    }
  }

  if (module.running() && (millis() - last_report_time > 50)) {
    uint8_t out_buf[2] = {0};
    out_buf[0] = BAB_CMD_REPORT_STATE;
    out_buf[1] = 0;
    for(unsigned int i = 0; i < 4; ++i) {
      out_buf[1] += (1-digitalRead(i)) << i;
    }
    module.sendPacket(out_buf, 2);
    last_report_time = millis();
  }
}

