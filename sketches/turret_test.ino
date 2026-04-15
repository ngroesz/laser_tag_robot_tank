#include <Wire.h>
#include <PVision.h>

#include "src/tank.h"

#define CAMERA_READ_DELAY 1000
#define CAMERA_MIDDLE_X 500
#define CAMERA_MOVE_THRESHOLD 50

unsigned long last_camera_read = 0;
PVision ircam;
byte result;

Tank tank;

// remove me
int current_motor_direction = 1;
unsigned long last_motor_direction_millis = 0;
#define MOTOR_CHANGE_DELAY 5000

void setup() {
#ifdef DEBUG_OUTPUT
  Serial.begin(115200);
  Serial.println(F("Initializing ..."));
#endif

  tank.initialize();
#ifdef DEBUG_OUTPUT
  Serial.println(F("Tank initialized."));
#endif

#ifdef CAMERA_ENABLED
  ircam.init();
#ifdef DEBUG_OUTPUT
  Serial.println(F("Camera initialized."));
#endif
#endif

#ifdef DEBUG_OUTPUT
  Serial.println(F("Finished initialization."));
#endif

  //digitalWrite(LEFT_MOTOR_PWM_PIN, 0);
  //digitalWrite(RIGHT_MOTOR_PWM_PIN, 0);
  //digitalWrite(MOTOR_ENABLE_PIN, 0);
  //digitalWrite(SHIFT_CLEAR_PIN, 0);
  //digitalWrite(SHIFT_CLOCK_PIN, 0);
  //digitalWrite(SHIFT_DATA_PIN, 0);

  //pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  //pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  //pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  //pinMode(SHIFT_CLEAR_PIN, OUTPUT);
  //pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  //pinMode(SHIFT_DATA_PIN, OUTPUT);
}

void loop() {
  unsigned long current_millis = millis();

//  if (last_camera_read + CAMERA_READ_DELAY < current_millis) {
//    result = ircam.read();
//    if (result & BLOB1) {
//#ifdef DEBUG_OUTPUT
//      Serial.print(F("BLOB1 detected. X:"));
//      Serial.print(ircam.Blob1.X);
//      Serial.print(F(" Y:"));
//      Serial.print(ircam.Blob1.Y);
//      Serial.print(F(" Size:"));
//      Serial.println(ircam.Blob1.Size);
//#endif
//      if (ircam.Blob1.X < CAMERA_MIDDLE_X - CAMERA_MOVE_THRESHOLD) {
//#ifdef DEBUG_OUTPUT
//        Serial.println(F("turret left"));
//#endif
//        tank.turret_left();
//      } else if(ircam.Blob1.X > CAMERA_MIDDLE_X + CAMERA_MOVE_THRESHOLD) {
//#ifdef DEBUG_OUTPUT
//        Serial.println(F("turret right"));
//#endif
//        tank.turret_right();
//      } else {
//#ifdef DEBUG_OUTPUT
//        Serial.print(F("turret stop"));
//#endif
//        tank.turret_stop();
//      }
//    }
//    last_camera_read = current_millis;
//  }

  if (last_motor_direction_millis + MOTOR_CHANGE_DELAY < current_millis) {
    if (current_motor_direction == 1) {
      Serial.println("directon change left");
      tank.turret_left();
      current_motor_direction = 2;
    } else {
      Serial.println("directon change right");
      tank.turret_right();
      current_motor_direction = 1;
    }
    last_motor_direction_millis = current_millis;
  }
  tank.loop();

  //unsigned char forward_code = CONTROL_CODE_TURRET_MOTOR_FORWARD;
  //unsigned char reverse_code = CONTROL_CODE_TURRET_MOTOR_REVERSE;
  //Serial.println("reverse");
  //analogWrite(TURRET_MOTOR_PWM_PIN, 255);
  //digitalWrite(MOTOR_ENABLE_PIN, 1);
  //write_motor_code(reverse_code);
  //delay(500);
}

void write_motor_code(const unsigned char & control_code) {
    Serial.print("Writing motor control code ");
    Serial.println(control_code);
    digitalWrite(SHIFT_CLEAR_PIN, LOW);
    shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, control_code);
    digitalWrite(SHIFT_CLEAR_PIN, HIGH);
}
