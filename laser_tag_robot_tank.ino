#include "tank.h"

#define TURRET_MOTOR_LEFT_PIN 8
#define TURRET_MOTOR_RIGHT_PIN 7
#define TURRET_ENCODER_PIN 13
#define TURRET_CALIBRATION_PIN 14

unsigned long current_millis;
unsigned long last_output_millis = 0;
unsigned long last_turret_change_millis = 0;

Tank tank(
    TURRET_MOTOR_LEFT_PIN,
    TURRET_MOTOR_RIGHT_PIN,
    TURRET_ENCODER_PIN,
    TURRET_CALIBRATION_PIN
);

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
