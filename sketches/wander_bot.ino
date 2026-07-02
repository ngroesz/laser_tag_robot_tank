/* wander_bot.ino
A bot that roams around and avoids obstacles. It uses the distance sensor
to build a 360-degree view and tracks the distance to the nearest obstacle
that exists within 8 45-degree "radar" slices.
The robot is a pacifist and does not shoot targets nor seek them.
*/

#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "constants.h"
#include "src/tank.h"

#define SENSOR_READ_DELAY 100
#define MAX_VALID_DISTANCE 8189

Tank tank;
VL53L0X sensor;

typedef void (*ModeFunction) ();
unsigned long last_distance_update_millis = 0;
uint16_t last_ir_command;
ModeFunction mode_function;

// radar array tracks nearest obstacle within a 45 degree slice
// radar[0]: [-22.5,22.5), radar[1]: [22.5,45), etc
uint16_t radar[8] = {0};

bool find_distance(uint16_t & distance)
{
  uint16_t sensor_read = sensor.readRangeContinuousMillimeters();

  if (sensor_read > 0 && sensor_read < DISTANCE_MAX) {
    Serial.println(sensor_read);
    distance = sensor_read;
    return true;
  }

  return false;
}


void setup()
{
#ifdef DEBUG_OUTPUT
  Serial.begin(115200);
  Serial.println(F("Initializing ..."));
#endif

  Wire.begin();

  tank.initialize();

  sensor.setTimeout(500);
  if (!sensor.init()) {
#ifdef DEBUG_OUTPUT
    Serial.println(F("Failed to detect and initialize sensor!"));
#endif
    while (1) {}
  }
  sensor.startContinuous(SENSOR_READ_DELAY);
#ifdef DEBUG_OUTPUT
  Serial.println(F("Distance sensor initialized."));
#endif

  tank.setup_routine();

  Serial.println("Setup routine has completed");

  // the first thing we want to do is reconnoiter
  mode_function = begin_recon;
}


void loop()
{
  // if mode_function is set, call it
  // if you want to the bot to be in idle mode, set mode_function to NULL
  if (mode_function) {
    mode_function();
  }

  tank.loop();
}

void begin_recon()
{
  Serial.println("starting recon");
  // after the turret spins 360 degrees, the start_wander() function will be called
  mode_function = recon;
  tank.turret_left_degrees(360, start_wander);
}

void recon()
{
  if (millis() > last_distance_update_millis + SENSOR_READ_DELAY) {
    uint16_t distance;
    if (find_distance(distance)) {
#ifdef DEBUG_OUTPUT
      Serial.print("distance: ");
      Serial.println(distance);
#endif
      // assuming the turret has been calibrated, turret_get_degrees returns a number between 0 and 359, inclusive
      int16_t degrees = tank.turret_get_degrees();
      int8_t radar_index = floor(tank.normalize_angle(degrees + 22.5) / 45);
      radar[radar_index] = distance;
    }
    last_distance_update_millis = millis();
  }
}

void start_wander()
{
  mode_function = NULL;
  Serial.println("ready to wander");
  Serial.println("contents of radar[]:");
  for (int i = 0; i < 8; ++i) {
    Serial.println(radar[i]);
  }
}

void wander()
{
  //if (millis() > last_distance_update_millis + SENSOR_READ_DELAY) {
  //  uint16_t distance;
  //  if (find_distance(distance)) {
  //    #ifdef DEBUG_OUTPUT
  //    Serial.print("distance: ");
  //    Serial.println(distance);
  //    #endif
  //  }
  //  last_distance_update_millis = millis();
  //}
}
