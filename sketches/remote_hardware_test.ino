#include "tank.h"

Tank tank;

byte last_ir_code;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing ...");
    tank.setup();
    Serial.println("Tank initialized.");
}

void loop()
{
    tank.loop();

    last_ir_code = tank.last_ir_code();

    switch (last_ir_code) {
        case IR_CODE_UP:
            tank.drive_forward();
            break;
        case IR_CODE_SELECT:
            tank.drive_stop();
            break;
        case IR_CODE_DOWN:
            tank.drive_reverse();
            break;
        case IR_CODE_RIGHT:
            tank.drive_turn_right();
            break;
        case IR_CODE_LEFT:
            tank.drive_turn_left();
            break;
    }
}
