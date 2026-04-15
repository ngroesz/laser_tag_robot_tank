#include "src/pins.h"
#include "src/tank.h"
#include "src/tank_constants.h"
#include "src/tank_led.h"


//TankLed _tank_led;
Tank tank;

void setup()
{
  uint8_t pins[] = {0, 1, 14};
  //_tank_led.setup(pins);
  tank.initialize();
}

uint32_t last_update_millis = 0;
uint8_t blink_delay = 1000;
boolean leds_on = false;

void loop()
{
  //if (last_update_millis + blink_delay < millis()) {
  //  if (leds_on) {
  //    _tank_led.all_off();
  //    leds_on = false;
  //  } else {
  //    _tank_led.all_on();
  //    leds_on = true;
  //  }
  //  last_update_millis = millis();
  //}

  //_tank_led.loop();
  tank.loop();
}
