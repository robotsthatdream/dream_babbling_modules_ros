#include <base_arduino_module.h>

#define BUTTON_PIN 2
#define LED_PIN 6
#define JOYSTICK_X_PIN A1
#define JOYSTICK_Y_PIN A2

BabblingModule module(BAB_MODULE_TYPE_JOYSTICK);

volatile int last_report_time;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
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
    uint8_t out_buf[6] = {0};
    out_buf[0] = BAB_CMD_REPORT_STATE;
    out_buf[1] = digitalRead(BUTTON_PIN) == 0 ? 1 : 0;
    uint16_t a = analogRead(JOYSTICK_X_PIN);
    memcpy(&out_buf[2],&a,sizeof(uint16_t));
    a = analogRead(JOYSTICK_Y_PIN);
    memcpy(&out_buf[4],&a,sizeof(uint16_t));
    module.sendPacket(out_buf, 6);
    last_report_time = millis();
  }
}

