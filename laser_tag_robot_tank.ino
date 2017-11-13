#include "PinChangeInterrupt.h"

const int turret_motor_pins[] = {8, 7};

int turret_encoder_pin = 13;
int turret_calibration_pin = 14;

short turret_direction = 0;
volatile long turret_encoder_count = 0;
volatile int turret_position = 0;
volatile bool turret_has_been_calibrated = false;

unsigned long current_millis;
unsigned long last_output_millis = 0;

unsigned long last_turret_calibration_millis = 0;
const int turret_calibration_delay_millis = 2000;

unsigned long last_turret_change_millis = 0;

const int turret_gear_ratio = 24000;

void turret_stop()
{
    analogWrite(turret_motor_pins[0], 0);
    analogWrite(turret_motor_pins[1], 0);
}

void turret_left()
{
    if (turret_direction != -1) {
        turret_stop();
    }
    analogWrite(turret_motor_pins[0], 255);
    analogWrite(turret_motor_pins[1], 0);
    turret_direction = -1;
}

void turret_right()
{
    if (turret_direction != 1) {
        turret_stop();
    }
    analogWrite(turret_motor_pins[0], 0);
    analogWrite(turret_motor_pins[1], 255);
    turret_direction = 1;
}

void turret_calibration_interrupt()
{
    if (current_millis < last_turret_calibration_millis + turret_calibration_delay_millis) {
        return;
    }

    turret_has_been_calibrated = true;
    turret_position = 0;
    turret_encoder_count = 0;
    last_turret_calibration_millis = current_millis;
}

void turret_encoder_interrupt()
{
    //turret_position += 1;
    turret_encoder_count += turret_direction;
    //Serial.println("encoder interrupt");
    //turret_position += 1;
}

void calculate_turret_position()
{
    if (!turret_has_been_calibrated) {
        return;
    }

    turret_position = int(turret_encoder_count / (turret_gear_ratio / 360));

    if (turret_position > 360) {
        turret_position -= 360;
    } else if(turret_position < 360) {
        turret_position += 360;
    }
}
void setup()
{
    Serial.begin(9600);
    pinMode(turret_motor_pins[0], OUTPUT);
    pinMode(turret_motor_pins[1], OUTPUT);

    pinMode(turret_encoder_pin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(turret_encoder_pin), turret_encoder_interrupt, CHANGE);

    pinMode(turret_calibration_pin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(turret_calibration_pin), turret_calibration_interrupt, RISING);

    turret_left();

    randomSeed(analogRead(0));
}

void loop()
{
    current_millis = millis();

    if (current_millis >= last_output_millis + 1000) {
        if (turret_has_been_calibrated) {
            Serial.print("turret position: " );
            Serial.println(turret_position);
        }
        //Serial.print(" turret encoder count: " );
        //Serial.println(turret_encoder_count);
        last_output_millis = current_millis;
    }

    if (turret_has_been_calibrated && current_millis >= last_turret_change_millis + 20000) {
        if (random(2) > 0) {
            if (turret_direction == -1) {
                turret_right();
            } else {
                turret_left();
            }
        }
        last_turret_change_millis = current_millis;
    }

    calculate_turret_position();
}
