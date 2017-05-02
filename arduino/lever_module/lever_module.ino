#include <base_arduino_module.h>

#define LEVER_PIN A1
#define LED_PIN 6

BabblingModule module(BAB_MODULE_TYPE_LEVER);

volatile int last_report_time;

void setup() {
  pinMode(LEVER_PIN, INPUT);
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
    uint8_t out_buf[3] = {0};
    out_buf[0] = BAB_CMD_REPORT_STATE;
    uint16_t a = analogRead(LEVER_PIN);
    memcpy(&out_buf[1],&a,sizeof(uint16_t));
    module.sendPacket(out_buf, 3);
    last_report_time = millis();
  }
}

