#include "tank.h"

#define DISTANCE_WARNING 10

#define LED_PIN_1 4
#define LED_PIN_2 A3
#define LED_PIN_3 6
#define LED_PIN_4 A0
#define IR_RECEIVER_PIN 2
#define MOTOR_ENABLE_PIN 5
#define TURRET_ENCODER_PIN 13
#define TURRET_CALIBRATION_PIN 14
#define SONAR_FRONT_PIN A1
#define SONAR_REAR_PIN A2

unsigned long current_millis;
unsigned long last_output_millis = 0;
unsigned long last_turret_change_millis = 0;

unsigned long last_spin_millis = 0;
int spin_delay = 2000;

Tank tank(
    IR_RECEIVER_PIN,
    MOTOR_ENABLE_PIN,
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
    Serial.begin(9600);

    Serial.println("Initialized.");
    digitalWrite(6, HIGH);
}

void loop()
{
    tank.do_loop();
    current_millis = millis();
}
