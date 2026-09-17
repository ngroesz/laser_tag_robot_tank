#include <Arduino.h>

#include "mini_led.h"

MiniLed::MiniLed(const uint8_t led_pins[], const uint8_t led_count, const boolean on_state)
{
  _led_count = led_count;
  _on_state = on_state;
  _off_state = _on_state == HIGH ? LOW : HIGH;
  for(uint8_t led_index = 0; led_index < _led_count; ++led_index) {
    Serial.print("initialize with pin: ");
    Serial.println(led_pins[led_index]);
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
  Serial.print("led pin ");
  Serial.println(led.led_pin);
  Serial.print("state ");
  Serial.println(_on_state);
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
  led_to_update.last_change_millis = millis();
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

//void MiniLed::set_blinks(uint8_t led_index, const uint16_t blinks[], const uint8_t blinks_size, const uint8_t max_blinks)
//{
//  Led& led_to_update = _leds[led_index];
//  led_to_update.max_blinks = max_blinks;
//
//  set_blinks(led_index, blinks, blinks_size);
//}
//
//void MiniLed::set_blinks(uint8_t led_index, const uint16_t blinks[], const uint8_t blinks_size)
//{
//  Led& led_to_update = _leds[led_index];
//
//  led_to_update.state = led_blinking;
//  led_to_update.blinks_size = blinks_size > MAX_BLINKS_SIZE ? MAX_BLINKS_SIZE : blinks_size;
//  for (uint8_t i = 0; i < led_to_update.blinks_size; ++i) {
//    led_to_update.blinks[i] = blinks[i];
//  }
//  led_to_update.current_blink_index = -1;
//  led_to_update.next_blink_change = 0;
//}

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
  Serial.print("pin mode: ");
  Serial.println(_led.led_pin);
  pinMode(_led.led_pin, OUTPUT);
  digitalWrite(_led.led_pin, _off_state);
}

//void MiniLed::_led_on(struct Led & _led)
//{
//  _led.state = led_on;
//  Serial.print("led pin ");
//  Serial.println(_led.led_pin);
//  Serial.print("state ");
//  Serial.println(_on_state);
//  digitalWrite(_led.led_pin, _on_state);
//}
//
//void MiniLed::_led_off(struct Led & _led)
//{
//  _led.state = led_off;
//  digitalWrite(_led.led_pin, _off_state);
//}

void MiniLed::_update_led(struct Led & _led)
{
  if (_current_millis > _led.last_change_millis + BLINK_MILLIS) {
    Serial.print("current_millis: ");
    Serial.println(_current_millis);
  
    //_print_led_state(_led);

    ++_led.blink_count;

    if(_led.blink_count % 2 == 0) {
      Serial.println("led on");
      // turn the LED on
      digitalWrite(_led.led_pin, _on_state);
    } else {
      Serial.println("led off");
      // turn the LED off
      digitalWrite(_led.led_pin, _off_state);

      // if we've reached max_blinks, re-initialize the LED, thus ending blinks
      if (_led.target_blinks > 0 && _led.blink_count >= _led.target_blinks * 2) {
        Serial.print("blink count: ");
        Serial.println(_led.blink_count);
        _reset_led(_led);
      }
    }
  }
}

//// TODO: remove me
//void MiniLed::_print_led_state(struct Led & _led)
//{
//  Serial.print("State of LED for pin: ");
//  Serial.println(_led.led_pin);
//
//  Serial.print("blink_count: ");
//  Serial.println(_led.blink_count);
//  
//  Serial.print("max_blinks: ");
//  Serial.println(_led.max_blinks);
//
//  Serial.print("current_blink_index: ");
//  Serial.println(_led.current_blink_index);
//
//  Serial.print("next_blink_change: ");
//  Serial.println(_led.current_blink_index);
//
//  Serial.print("blinks_size: ");
//  Serial.println(_led.blinks_size);
//
//  Serial.print("blinks: ");
//  for (int i = 0; i < _led.blinks_size; ++i) {
//    Serial.print(_led.blinks[i]);
//    Serial.print("\t");
//  }
//  Serial.println("");
//}
