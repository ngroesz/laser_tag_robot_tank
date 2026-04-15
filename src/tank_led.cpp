#include <Arduino.h>

#include "tank_led.h"

void TankLed::setup(const uint8_t led_pins[])
{
  for(uint8_t led_index = 0; led_index < LED_COUNT; ++led_index) {
    _initialize_led(_leds[led_index], led_pins[led_index]);
  }
}

void TankLed::loop()
{
  _current_millis = millis();

  for(uint8_t led_index = 0; led_index < LED_COUNT; ++led_index) {
    if (_leds[led_index].state == blinking) {
      _update_led(_leds[led_index]);
    }
  }
}

// TODO: I'm not sure if it's necessary to put the actual logic into the private function
void TankLed::turn_on(uint8_t led_index) { _turn_led_on(_leds[led_index]); }
void TankLed::turn_off(uint8_t led_index) { _turn_led_off(_leds[led_index]); }

void TankLed::toggle(uint8_t led_index)
{
  if (_leds[led_index].state == on) {
    turn_off(led_index);
  } else {
    turn_on(led_index);
  }
}

void TankLed::all_on()
{
  for(uint8_t led_index = 0; led_index < LED_COUNT; ++led_index) {
    turn_on(led_index);
  }
}

void TankLed::all_off()
{
  for(uint8_t led_index = 0; led_index < LED_COUNT; ++led_index) {
    turn_off(led_index);
  }
}

void TankLed::led_set_blinks(uint8_t led_index, const uint16_t blinks[], const uint8_t blink_count)
{
  Led led_to_update = _leds[led_index];

  led_to_update.state = blinking;
  led_to_update.blink_count = blink_count > 10 ? 10 : blink_count;
  for (uint8_t i = 0; i <= led_to_update.blink_count; ++i) {
    led_to_update.blinks[i] = blinks[i];
  }
  led_to_update.current_blink_index = -1;
  led_to_update.next_blink_change = 0;
}

void TankLed::_initialize_led(struct Led & _led, const uint8_t led_pin)
{
  _led.led_pin = led_pin;
  _led.state = off;
  _led.blink_count = 0;
  pinMode(_led.led_pin, OUTPUT);
  digitalWrite(_led.led_pin, HIGH);
}

void TankLed::_turn_led_on(struct Led & _led)
{
  _led.state = on;
  digitalWrite(_led.led_pin, LOW);
}

void TankLed::_turn_led_off(struct Led & _led)
{
  _led.state = off;
  digitalWrite(_led.led_pin, HIGH);
}

void TankLed::_update_led(struct Led & _led)
{
  if (_current_millis > _led.next_blink_change) {
    ++_led.current_blink_index;
    if (_led.current_blink_index == _led.blink_count) {
      _led.current_blink_index = 0;
    }
    if(_led.current_blink_index % 2 == 0) {
      // turn the LED on
      digitalWrite(_led.led_pin, LOW);
    } else {
      // turn the LED off
      digitalWrite(_led.led_pin, HIGH);
   }
   _led.next_blink_change = _current_millis + _led.blinks[_led.current_blink_index];
  }
}
