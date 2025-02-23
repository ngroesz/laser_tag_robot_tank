#include "tank.h"

Tank tank;

byte last_ir_code;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing hardware_test ...");
    tank.setup();
    Serial.println("Tank initialized.");
}

unsigned long drive_change_millis = 0;
unsigned short drive_direction = 0;

void loop()
{
    tank.loop();

    if (drive_change_millis < millis()) {
        Serial.println("switch direction");
      if (drive_direction == 0) {
        Serial.println("forward!");
        tank.drive_forward();
        drive_direction = 1;
      } else {
        Serial.println("backward!");
        tank.drive_reverse();
        drive_direction = 0;
      }
      drive_change_millis = millis() + 3000;
    }
}
