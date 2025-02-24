// the simplest possible motor driver, using a shift register
// just make the motors go forward and backward, operating on a delay

#include "constants.h"
#include "pins.h"

unsigned char forward_code = 0 | CONTROL_CODE_LEFT_MOTOR_FORWARD | CONTROL_CODE_RIGHT_MOTOR_FORWARD;
unsigned char reverse_code = 0 | CONTROL_CODE_LEFT_MOTOR_REVERSE | CONTROL_CODE_RIGHT_MOTOR_REVERSE;

void setup() {
    Serial.begin(115200);

    digitalWrite(LEFT_MOTOR_PWM_PIN, 0);
    digitalWrite(RIGHT_MOTOR_PWM_PIN, 0);
    digitalWrite(MOTOR_ENABLE_PIN, 0);
    digitalWrite(SHIFT_CLEAR_PIN, 0);
    digitalWrite(SHIFT_CLOCK_PIN, 0);
    digitalWrite(SHIFT_DATA_PIN, 0);

    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(SHIFT_CLEAR_PIN, OUTPUT);
    pinMode(SHIFT_CLOCK_PIN, OUTPUT);
    pinMode(SHIFT_DATA_PIN, OUTPUT);

    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));
}

void loop() {
    // forward
    Serial.println("forward");
    analogWrite(LEFT_MOTOR_PWM_PIN, 255);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 255);
    digitalWrite(MOTOR_ENABLE_PIN, 1);
    write_motor_code(forward_code);
    delay(5000);

    // stop
    Serial.println("stop");
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
    digitalWrite(MOTOR_ENABLE_PIN, 0);
    write_motor_code(0);
    delay(1000);

    // reverse
    Serial.println("reverse");
    analogWrite(LEFT_MOTOR_PWM_PIN, 255);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 255);
    digitalWrite(MOTOR_ENABLE_PIN, 1);
    write_motor_code(reverse_code);
    delay(5000);
}

void write_motor_code(const unsigned char & control_code) {
    Serial.print("Writing motor control code ");
    Serial.println(control_code);
    digitalWrite(SHIFT_CLEAR_PIN, LOW);
    shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, control_code);
    digitalWrite(SHIFT_CLEAR_PIN, HIGH);
}
