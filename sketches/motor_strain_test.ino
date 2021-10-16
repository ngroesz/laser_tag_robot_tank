/*
This sketch is intended to make the motors work hard.
It is good for testing thermal properties of motor controllers.
Also a test of basic low-level motor controller code
*/

#include <Wire.h>
#include <PVision.h>

#include "tank.h"

unsigned long last_update_millis;

Tank tank;

const char LEFT_TURN = 0;
const char RIGHT_TURN = 1;
const char FORWARD = 2;
const char REVERSE = 3;

const unsigned long STATE_CHANGE_DELAY = 5000;

char current_state = LEFT_TURN;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing ...");
    tank.setup();
    Serial.println("Tank initialized.");
    tank.enable_motors(1);

    last_update_millis = 0;
}

void loop()
{
    unsigned long current_millis = millis();

    if (current_millis > last_update_millis + STATE_CHANGE_DELAY) {
        switch (current_state) {
            case LEFT_TURN:
                current_state = RIGHT_TURN;
                tank.drive_turn_right();
                break;
            case RIGHT_TURN:
                current_state = FORWARD;
                tank.drive_forward();
                break;
            case FORWARD:
                current_state = REVERSE;
                tank.drive_reverse();
                break;
            case REVERSE:
                current_state = LEFT_TURN;
                tank.drive_turn_left();
                break;
        }
        last_update_millis = current_millis;
    }

    tank.loop();
}
