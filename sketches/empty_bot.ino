/* empty_bot.ino
Bot initializes libraries but does absolutely nothing. It's a good starting
template for your own bot.
*/

// You will need the following libraries in order to use the camera and distance sensor
#include <PVision.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "src/tank.h"

Tank tank; // declare tank library
PVision ircam; // declare ir camera library
VL53L0X sensor; // declare distance sensor library

constexpr uint16_t camera_read_delay = 100;
constexpr uint16_t distance_sensor_read_delay = 100;

struct Target {
  bool active = false;
  uint16_t x;
  uint16_t y;
};

// put global variables here
Target target;
uint32_t last_camera_update_millis = 0;

void setup()
{
  // You will need to initialize Serial if you want to read debugging output over an FTDI interface
  // You can remove Serial output for your final version or you can have some constant like
  // DEBUG_OUTPUT that controls whether or not Serial statements are compiled into your program.
  Serial.begin(115200);

  // It can be useful to include filenames and modification times in debug output.
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));

  // The F() macro will cause strings to be saved in flash memory, as opposed to SRAM.
  // Since SRAM is very limited, this can be useful, particularily if you have a
  // lot of debugging statements.
  Serial.println(F("Empty bot initializing ..."));

  // you have to call this function in order to initialize i2c communications
  Wire.begin();

  tank.initialize();

  // call this function if the front bumper is pressed
  tank.set_bump_front_callback(bump_front);

  // call this function if the front bumper is pressed
  tank.set_bump_rear_callback(bump_rear);

  ircam.init();
#ifdef DEBUG_OUTPUT
  Serial.println("Camera initialized.");
#endif

  sensor.setTimeout(500);
  if (!sensor.init()) {
#ifdef DEBUG_OUTPUT
    Serial.println("Failed to detect and initialize sensor!");
#endif
    while (1) {}
  }
  sensor.startContinuous(distance_sensor_read_delay);
#ifdef DEBUG_OUTPUT
  Serial.println("Distance sensor initialized.");
#endif
  }

  // put your custom setup logic here

  // this will do the setup routine which waits for OK, calibrates the turret, and then waits for OK again
  // this is how a game-playing robot should start but it could be annoying for standard development so
  // feel free to comment it out until you create your competition robot
  tank.setup_routine();
}

void loop()
{
  // this call must always be left in your loop()
  tank.loop();

  // check to see if the camera has spotted a target
  if (millis() > last_camera_update_millis + camera_read_delay) {
    if (find_target(my_target)) {
      Serial.println("target spotted");
    }

    last_camera_update_millis = millis();
  }
  // here you can do logic or call functions
}


bool find_target(struct target & t)
{
  byte result = ircam.read();

  if (result & BLOB1) {
      t.x = ircam.Blob1.X;
      t.y = ircam.Blob1.Y;
      t.size = ircam.Blob1.Size;
      return true;
  }

  return false;
}

void bump_front()
{
  // the front bumper has been pressed, think about doing some obstacle avoidance
}

void bump_rear()
{
  // the rear bumper has been pressed, think about doing some obstacle avoidance
}
