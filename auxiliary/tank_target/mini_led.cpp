#include <Arduino.h>

#include "mini_led.h"

void MiniLed::setup(const uint8_t led_pins[], const uint8_t led_count, const boolean on_state)
{
  _led_count = led_count;
  _on_state = on_state;
  _off_state = _on_state == HIGH ? LOW : HIGH;
  for(uint8_t led_index = 0; led_index < _led_count; ++led_index) {
    _initialize_led(_leds[led_index], led_pins[led_index]);
  }
}

void MiniLed::loop()
{
  _current_millis = millis();

  for(uint8_t led_index = 0; led_index < _led_count; ++led_index) {
    if (_leds[led_index].state == led_blinking) {
      _update_led(_leds[led_index]);
    }
  }
}

void MiniLed::on(uint8_t led_index)
{
  Led led = _leds[led_index];

  led.state = led_on;
  digitalWrite(led.led_pin, _on_state);
}

void MiniLed::off(uint8_t led_index)
{
  Led led = _leds[led_index];

  led.state = led_off;
  digitalWrite(led.led_pin, _off_state);
}

void MiniLed::toggle(uint8_t led_index)
{
  if (_leds[led_index].state == led_on) {
    off(led_index);
  } else {
    on(led_index);
  }
}

void MiniLed::blink(const uint8_t led_index, const uint8_t blink_count = 0)
{
  Led& led_to_update = _leds[led_index];
  led_to_update.state = led_blinking;
  led_to_update.blink_count = 0;
  led_to_update.target_blinks = blink_count;
  led_to_update.last_change_millis = millis();
  on(led_index);
}

void MiniLed::all_on()
{
  for(uint8_t led_index = 0; led_index < _led_count; ++led_index) {
    on(led_index);
  }
}

void MiniLed::all_off()
{
  for(uint8_t led_index = 0; led_index < _led_count; ++led_index) {
    off(led_index);
  }
}

void MiniLed::_initialize_led(struct Led & _led, const uint8_t led_pin)
{
  _led.led_pin = led_pin;
  _reset_led(_led);
}

void MiniLed::_reset_led(struct Led & _led)
{
  _led.state = led_off;
  _led.blink_count = 0;
  _led.target_blinks = 0;
  _led.last_change_millis;
  pinMode(_led.led_pin, OUTPUT);
  digitalWrite(_led.led_pin, _off_state);
}

void MiniLed::_update_led(struct Led & _led)
{
  if (_current_millis > _led.last_change_millis + BLINK_MILLIS) {
    _led.last_change_millis = _current_millis;

    ++_led.blink_count;

    if(_led.blink_count % 2 == 0) {
      // turn the LED on
      digitalWrite(_led.led_pin, _on_state);
    } else {
      // turn the LED off
      digitalWrite(_led.led_pin, _off_state);

      // if we've reached max_blinks, re-initialize the LED, thus ending blinks
      if (_led.target_blinks > 0 && _led.blink_count >= (_led.target_blinks * 2) - 1) {
        _reset_led(_led);
      }
    }
  }
}

// TODO: remove me
void MiniLed::_print_led_state(struct Led & _led)
{
  Serial.print("LED state for pin: ");
  Serial.println(_led.led_pin);

  Serial.print("State: ");
  Serial.println(_led.state);

  Serial.print("blink_count: ");
  Serial.println(_led.blink_count);

  Serial.print("target_blinks: ");
  Serial.println(_led.target_blinks);

  Serial.print("last_change_millis: ");
  Serial.println(_led.last_change_millis);
}
