#include <Arduino.h>

#include "tank_led.h"

void TankLED::setup(const uint8_t led_pins[])
{
    for(uint8_t led_index = 0; led_index < LED_COUNT; ++led_index) {
        _initialize_led(_led_states[led_index], led_pins[led_index]);
    }
}

void TankLED::loop()
{
    _current_millis = millis();

    for(uint8_t led_index = 0; led_index < LED_COUNT; ++led_index) {
        _update_led(_led_states[led_index]);
    }
}

void TankLED::led_turn_on(uint8_t led_index) { _turn_led_on(_led_states[led_index]); }
void TankLED::led_turn_off(uint8_t led_index) { _turn_led_off(_led_states[led_index]); }

void TankLED::all_on()
{
    for(uint8_t led_index = 0; led_index < LED_COUNT; ++led_index) {
        led_turn_on(led_index);
    }
}

void TankLED::all_off()
{
    for(uint8_t led_index = 0; led_index < LED_COUNT; ++led_index) {
        led_turn_off(led_index);
    }
}

void TankLED::led_set_state(uint8_t led_index, const uint16_t states[], const uint8_t state_count)
{
    _led_set_state(_led_states[led_index], states, state_count);
}

void TankLED::_initialize_led(struct led_state & _led_state, const uint8_t led_pin)
{
    _led_state.led_pin = led_pin;
    _led_state.led_is_on_or_off = true;
    _led_state.state_count = 0;
    pinMode(_led_state.led_pin, OUTPUT);
    digitalWrite(_led_state.led_pin, HIGH);
}

void TankLED::_turn_led_on(struct led_state & _led_state)
{
    _led_state.led_is_on_or_off = true;
    digitalWrite(_led_state.led_pin, LOW);
}

void TankLED::_turn_led_off(struct led_state & _led_state)
{
    _led_state.led_is_on_or_off = true;
    digitalWrite(_led_state.led_pin, HIGH);
}

void TankLED::_led_set_state(struct led_state & _led_state, const uint16_t states[], const uint8_t state_count)
{
    _led_state.led_is_on_or_off = false;
    _led_state.state_count = state_count > 10 ? 10 : state_count;
    for (uint8_t i = 0; i <= _led_state.state_count; ++i) {
        _led_state.states[i] = states[i];
    }
    _led_state.current_state = -1;
    _led_state.next_state_change = 0;
}

void TankLED::_update_led(struct led_state & _led_state)
{
    if (_led_state.led_is_on_or_off)  {
        return;
    }

    if (_current_millis > _led_state.next_state_change) {
        ++_led_state.current_state;
        if (_led_state.current_state == _led_state.state_count) {
            _led_state.current_state = 0;
        }
        if(_led_state.current_state % 2 == 0) {
            // turn the LED on
            digitalWrite(_led_state.led_pin, LOW);
        } else {
            // turn the LED off
            digitalWrite(_led_state.led_pin, HIGH);
        }
        _led_state.next_state_change = _current_millis + _led_state.states[_led_state.current_state];
    }
}
