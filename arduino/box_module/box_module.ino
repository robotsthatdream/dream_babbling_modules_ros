#include <base_arduino_module.h>
#include <Servo.h>

#define SERVO_PWM 2
#define LED_PIN 6

static unsigned int pw = 1000;
static Servo s;

BabblingModule module(BAB_MODULE_TYPE_BOX);

volatile int last_report_time;

void setup() {
  pinMode(SERVO_PWM, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);
  //while(!Serial);

  module.begin();
  s.attach(2);
  s.writeMicroseconds(pw);

  digitalWrite(LED_PIN, HIGH);

  last_report_time = millis();
}

void loop() {
  uint8_t in_buf[BAB_MAX_PACKET_SIZE];
  int packet_sz = module.getPacket(in_buf);

  if (packet_sz > 0) {
    switch (in_buf[0]) {
      case BAB_CMD_OPEN:
        pw = in_buf[1] == 1 ? 1700 : 1000;
        s.writeMicroseconds(pw);
        Serial.println(pw);
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }

  if (module.running() && (millis() - last_report_time > 50)) {
    uint8_t out_buf[2] = {0};
    out_buf[0] = BAB_CMD_REPORT_STATE;
    out_buf[1] = pw == 1000 ? 0 : 1;
    module.sendPacket(out_buf, 2);
    last_report_time = millis();
  }
}

