#include <base_arduino_module.h>

#define BUTTON_PIN 2
#define BUTTON_LED_PIN 3
#define LED_PIN 6

BabblingModule module(BAB_MODULE_TYPE_BUTTON);

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);
  //while(!Serial);

  module.begin();

  digitalWrite(LED_PIN, HIGH);
}

void loop() {
    uint8_t in_buf[BAB_MAX_PACKET_SIZE];
  int packet_sz = module.getPacket(in_buf);
  
  if (packet_sz > 0) {
    switch (in_buf[0]) {
      case BAB_CMD_LED:
        digitalWrite(BUTTON_LED_PIN, in_buf[1] == 0 ? LOW : HIGH);
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }  

  if (module.running()) {
    uint8_t out_buf[2] = {0};
    out_buf[0] = BAB_CMD_REPORT_STATE;
    out_buf[1] = digitalRead(BUTTON_PIN) == 0 ? 1 : 0;
    module.sendPacket(out_buf,2);
  }

  delay(1);
}

