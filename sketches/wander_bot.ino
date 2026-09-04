/* wander_bot.ino
A bot that roams around and avoids obstacles. It uses the distance sensor
to build a 360-degree view and tracks the distance to the nearest obstacle
that exists within 8 45-degree "radar" slices.
The robot is a pacifist and does not shoot targets nor seek them.
As of this writing, the wander bot does indeed wander around a room and
does a passable job at avoiding obstacles.
*/

/* TODO:
Need to add bump detection and subsequent obstacle avoidance
Could improve object distance tracking as noted in recon() function
*/

#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "constants.h"
#include "src/tank.h"

#define SENSOR_READ_DELAY 100
#define MAX_VALID_DISTANCE 8192

// Changing RADAR_SECTIONS to a higher number will allow for a more fine-grained view of obstacles,
// but I haven't tested this.
#define RADAR_SECTIONS 8

constexpr float radar_section_degrees = 360 / RADAR_SECTIONS;
constexpr float radar_section_fraction = (float)RADAR_SECTIONS / 360;
// using round() here because we don't need to deal in fractions of degrees
constexpr float radar_half_section_degrees = round(radar_section_degrees / 2);

Tank tank;
VL53L0X sensor;

typedef void (*ModeFunction) ();
unsigned long last_distance_update_millis = 0;
uint16_t last_ir_command;
ModeFunction mode_function;

// radar array tracks nearest obstacle within a 45 degree slice
// radar[0]: [-22.5,22.5), radar[1]: [22.5,45), etc
uint16_t radar[RADAR_SECTIONS] = {0};

bool find_distance(uint16_t & distance)
{
  uint16_t sensor_read = sensor.readRangeContinuousMillimeters();

  if (sensor_read > 0 && sensor_read < MAX_VALID_DISTANCE) {
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

  tank.set_ir_command_callback(process_ir_command);

  // the first thing we want to do is reconnoiter
  mode_function = begin_recon;

  Serial.println("finished setup");
}

// TODO: pause/unpause should be part of main tank library
void process_ir_command(uint16_t command)
{
  if (command == IR_CODE_POUND) {
    Serial.println("told to stop");
    tank.drive_stop();
    tank.turret_stop();
    mode_function = NULL;
  } else if (command = IR_CODE_ASTERISK) {
    Serial.println("resuming");
    mode_function = finish_recon;
  }
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
// TODO: could consider to make whether tank stops at target an optional flag

// to reconnoiter, take a 360-degree survey without driving
void begin_recon()
{
  Serial.println("starting recon");
  tank.drive_stop();
  // after the turret spins 360 degrees, the finish_recon() function will be called
  mode_function = recon;
  tank.turret_left_degrees(360, finish_recon);
}

void recon()
{
  if (millis() > last_distance_update_millis + SENSOR_READ_DELAY) {
    uint16_t distance;
    if (find_distance(distance)) {
      // Assuming the turret has been calibrated, turret_get_degrees returns a number between 0 and 359, inclusive.
      uint8_t radar_index = degrees_to_radar_index(tank.turret_get_degrees());

      // This is not a perfect system. As is, a narrow, nearby obstacle might be "forgotten" about because the last
      // reading within the section was further away.
      // I might have to revisit this. A more complex and probably more robust system would be to ensure that an entire
      // radar section has been traversed, and then and only then, set the distance to the CLOSEST distance found within
      // that section. Potentially this could involve some shortcut that would alert the system of a particularly close
      // object, without waiting for the entire section to be traversed.
      radar[radar_index] = distance;
    }
    last_distance_update_millis = millis();
  }
}

/// @brief Translate degrees to radar index.
/// @param degrees Degrees number. This number will be normalized to a degree value between 0 and 359.
/// @return The radar index corresponding to the degree value.
uint8_t degrees_to_radar_index(int16_t degrees)
{
  // this math is somewhat optimized by using multiplication instead of division.
  // i'm not sure how important it is to optimize in this way.
  return floor(tank.normalize_angle(degrees + radar_half_section_degrees) * (radar_section_fraction));
}

/// @brief Translate radar index to degrees.
/// @param radar_index Index of radar array.
/// @return The degree value corresponding to the center of the section represented by the index.
int16_t radar_index_to_degrees(uint8_t radar_index)
{
  // we subtract HALF_SECTION_DEGREES in order to find the degree value that corresponds to the middle of the section
  return round(radar_index * radar_section_degrees - 22.5);
}

int16_t degrees_for_open_area()
{
  uint16_t furthest_distance = 0;
  int8_t index_of_furthest_distance;
  int radar_index;
  for (radar_index = 0; radar_index < RADAR_SECTIONS; ++radar_index) {
    if (radar[radar_index] > furthest_distance) {
      furthest_distance = radar[radar_index];
      index_of_furthest_distance = radar_index;
    }
  }

  // For whatever reason (perhaps radar not initialized?), there is no distance found. Thusly,
  // we return -1 to indicate that no open area was found.
  if (furthest_distance == 0) {
    return -1;
  }

  return radar_index_to_degrees(index_of_furthest_distance);
}

int8_t furthest_distance_on_radar()
{
  uint16_t furthest_distance = 0;
  for (int i = 0; i < RADAR_SECTIONS; ++i) {
    if (radar[i] > furthest_distance) {
      furthest_distance = radar[i];
    }
  }

  return furthest_distance;
}


void finish_recon()
{
  Serial.println("finished recon");
  turn_toward_open_area();
  // we dont want to do anything else until we've turned toward target
  mode_function = NULL;
}

void turn_toward_open_area()
{
  Serial.println("contents of radar[]:");
  for (int i = 0; i < 8; ++i) {
    Serial.println(radar[i]);
  }
  int16_t target_degrees = degrees_for_open_area();
  Serial.print("turning to target degrees ");
  Serial.println(target_degrees);
  tank.drive_turn_degrees(target_degrees, start_wander);
}

void start_wander()
{
  // we will be continuously scanning the area
  tank.turret_right();
  tank.drive_forward();
  mode_function = wander;
}

void wander()
{
  recon();
  if (radar[0] < 500 || radar[0] < 1000 && (furthest_distance_on_radar() > 5000)) {
    Serial.print("direction change at millis ");
    Serial.println(millis());
    turn_toward_open_area();
  }
}
