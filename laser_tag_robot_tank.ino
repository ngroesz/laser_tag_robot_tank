#include "tank.h"

const int turret_motor_pins[] = {8, 7};

int turret_encoder_pin = 13;
int turret_calibration_pin = 14;

unsigned long current_millis;
unsigned long last_output_millis = 0;
unsigned long last_turret_change_millis = 0;

volatile long _turret_encoder_count = 0;
volatile bool _turret_has_been_calibrated = false;
volatile unsigned long _last_turret_calibration_millis = 0;
short _turret_direction = 0;

Tank tank(8, 7, 13, 14);

void setup()
{
    Serial.begin(9600);

    randomSeed(analogRead(0));
}

void loop()
{
    current_millis = millis();

    if (current_millis >= last_output_millis + 1000) {
        if (tank.turret_has_been_calibrated()) {
            Serial.print("turret position: " );
            Serial.print(int(tank.turret_position()));
            Serial.print(" turret encoder: " );
            Serial.println(int(tank.turret_encoder_count()));
        }
        last_output_millis = current_millis;
    }

    if (!tank.turret_has_been_calibrated()) {
        tank.turret_left();
    }

    if (tank.turret_has_been_calibrated() && current_millis >= last_turret_change_millis + 20000) {
        if (random(2) > 0) {
            if (tank.turret_direction() == -1) {
                tank.turret_right();
            } else {
                tank.turret_left();
            }
        }
        last_turret_change_millis = current_millis;
    }
}
