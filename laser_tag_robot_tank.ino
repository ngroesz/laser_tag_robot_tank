#include "tank.h"

#define DISTANCE_WARNING 10

#define SHIFT_CLEAR_PIN 13
#define SHIFT_CLOCK_PIN 12
#define SHIFT_DATA_PIN 8
#define LED_PIN_1 4
#define LED_PIN_2 A3
#define LED_PIN_3 6
#define LED_PIN_4 A0
#define IR_RECEIVER_PIN 2
#define MOTOR_ENABLE_PIN 5
#define LEFT_MOTOR_PWM_PIN 9
#define RIGHT_MOTOR_PWM_PIN 10
#define TURRET_MOTOR_PWM_PIN 11
#define TURRET_ENCODER_PIN 13
#define TURRET_CALIBRATION_PIN 14
#define SONAR_FRONT_PIN A1
#define SONAR_REAR_PIN A2

unsigned long last_update_millis = 0;

Tank tank(
    IR_RECEIVER_PIN,
    MOTOR_ENABLE_PIN,
    LEFT_MOTOR_PWM_PIN,
    RIGHT_MOTOR_PWM_PIN,
    TURRET_MOTOR_PWM_PIN,
    SHIFT_CLEAR_PIN,
    SHIFT_CLOCK_PIN,
    SHIFT_DATA_PIN,
    TURRET_ENCODER_PIN,
    TURRET_CALIBRATION_PIN,
    SONAR_FRONT_PIN,
    SONAR_REAR_PIN,
    LED_PIN_1,
    LED_PIN_2,
    LED_PIN_3,
    LED_PIN_4
);

void setup()
{
    Serial.begin(115200);

    tank.setup();
    Serial.println("Tank initialized.");
    tank.enable_motors();
    randomSeed(analogRead(0));
}

void loop()
{
    unsigned long current_millis = millis();

    //if (current_millis > last_update_millis + 5000) {
    //    int speed = random(0, 155) + 100;
    //    tank.drive_forward(speed, 0);
    //    last_update_millis = current_millis;
    //}
    tank.loop();
}
