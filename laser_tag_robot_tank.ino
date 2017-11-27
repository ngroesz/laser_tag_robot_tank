#include "tank.h"

#define LED_PIN_1 4
#define LED_PIN_2 A3

#define TURRET_MOTOR_LEFT_PIN 8
#define TURRET_MOTOR_RIGHT_PIN 7
#define TURRET_ENCODER_PIN 13
#define TURRET_CALIBRATION_PIN 14

#define SONAR_FRONT_PIN A1
#define SONAR_REAR_PIN A2

unsigned long current_millis;
unsigned long last_output_millis = 0;
unsigned long last_turret_change_millis = 0;

Tank tank(
    TURRET_MOTOR_LEFT_PIN,
    TURRET_MOTOR_RIGHT_PIN,
    TURRET_ENCODER_PIN,
    TURRET_CALIBRATION_PIN,
    LED_PIN_1,
    LED_PIN_2,
    SONAR_FRONT_PIN,
    SONAR_REAR_PIN
);

void setup()
{
    Serial.begin(9600);

    randomSeed(analogRead(0));

    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);

    digitalWrite(LED_PIN_1, LOW);
    digitalWrite(LED_PIN_2, HIGH);
    delay(500);
    digitalWrite(LED_PIN_1, HIGH);
    digitalWrite(LED_PIN_2, LOW);
    delay(500);
    digitalWrite(LED_PIN_1, LOW);
    digitalWrite(LED_PIN_2, HIGH);
    delay(500);
    digitalWrite(LED_PIN_1, LOW);
    digitalWrite(LED_PIN_2, LOW);
}

void loop()
{
    tank.do_loop();
    current_millis = millis();

    if (current_millis >= last_output_millis + 1000) {
        //if (random(2) > 0) {
        //    tank.set_led_1(100, 1000);
        //    tank.set_led_2(1000, 100);
        //} else {
        //    tank.set_led_1(500, 500);
        //    tank.set_led_2(500, 500);
        //}
        Serial.print("front distance: " );
        Serial.print(tank.front_distance());
        Serial.print(" rear distance: " );
        Serial.println(tank.rear_distance());
        last_output_millis = current_millis;
    }

    //if (!tank.turret_has_been_calibrated()) {
    //    tank.turret_left();
    //}

    //if (tank.turret_has_been_calibrated() && current_millis >= last_turret_change_millis + 20000) {
    //    if (random(2) > 0) {
    //        if (tank.turret_direction() == -1) {
    //            tank.turret_right();
    //        } else {
    //            tank.turret_left();
    //        }
    //    }
    //    last_turret_change_millis = current_millis;
    //}
}
