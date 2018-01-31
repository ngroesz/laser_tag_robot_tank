#include <Wire.h>
#include <PVision.h>

#include "tank.h"

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

PVision ircam;

struct target {
    uint16_t x;
    uint16_t y;
    uint16_t size;
};

bool find_closest_target(struct target & t)
{
    byte result = ircam.read();
    uint8_t closest = 0;

    if (result & BLOB1) {
        t.x = ircam.Blob1.X;
        t.y = ircam.Blob1.Y;
        t.size = ircam.Blob1.Size;
        closest = t.size;
    }

    if (result & BLOB2 && ircam.Blob2.Size > closest) {
        t.x = ircam.Blob2.X;
        t.y = ircam.Blob2.Y;
        t.size = ircam.Blob2.Size;
        closest = t.size;
    }

    if (result & BLOB3 && ircam.Blob3.Size > closest) {
        t.x = ircam.Blob3.X;
        t.y = ircam.Blob3.Y;
        t.size = ircam.Blob3.Size;
        closest = t.size;
    }

    if (result & BLOB3 && ircam.Blob3.Size > closest) {
        t.x = ircam.Blob3.X;
        t.y = ircam.Blob3.Y;
        t.size = ircam.Blob3.Size;
        closest = t.size;
    }

    if (closest > 0) {
        return true;
    } else {
        return false;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing ...");
    tank.setup();
    Serial.println("Tank initialized.");
    //ircam.init();
    //Serial.println("Camera initialized.");
    tank.enable_motors(1);
    randomSeed(analogRead(0));
}

void loop()
{
    unsigned long current_millis = millis();

    //if (current_millis > last_update_millis + 500) {
    //    struct target my_target;
    //    if (find_closest_target(my_target)) {
    //        Serial.println("Target Detected");
    //        Serial.print(" X: ");
    //        Serial.print(my_target.x);
    //        Serial.print(" Y: ");
    //        Serial.print(my_target.y);
    //        Serial.print(" Size: ");
    //        Serial.println(my_target.size);
    //    }

    //    last_update_millis = current_millis;
    //}
    tank.loop();
}
