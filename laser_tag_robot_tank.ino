#include "tank.h"

#define LED_PIN_1 4
#define LED_PIN_2 A3

#define DISTANCE_WARNING 10

#define MOTOR_ENABLE_PIN 5
#define DRIVE_LEFT_PIN_1 9
#define DRIVE_LEFT_PIN_2 6
#define DRIVE_RIGHT_PIN_1 11
#define DRIVE_RIGHT_PIN_2 10
#define TURRET_MOTOR_PIN_1 8
#define TURRET_MOTOR_PIN_2 7
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
    MOTOR_ENABLE_PIN,
    DRIVE_LEFT_PIN_1,
    DRIVE_LEFT_PIN_2,
    DRIVE_RIGHT_PIN_1,
    DRIVE_RIGHT_PIN_2,
    TURRET_MOTOR_PIN_1,
    TURRET_MOTOR_PIN_2,
    TURRET_ENCODER_PIN,
    TURRET_CALIBRATION_PIN,
    LED_PIN_1,
    LED_PIN_2,
    SONAR_FRONT_PIN,
    SONAR_REAR_PIN
);

void setup()
{
    // first thing we do is make sure that the motors are disabled
    // they will be enabled later on
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

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

    //tank.enable_motors();
    Serial.println("Initialized.");
}

void loop()
{
    tank.do_loop();
    current_millis = millis();

    if (tank.rear_distance() < DISTANCE_WARNING) {
        last_spin_millis = current_millis;
    }

    if (last_spin_millis + spin_delay < current_millis) {
        tank.drive_turn_left();
    } else {
        tank.drive_reverse();
    }
}
