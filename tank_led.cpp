#include <Arduino.h>

#include "tank_led.h"

void TankLED::setup(
    const uint8_t led_pin_1,
    const uint8_t led_pin_2,
    const uint8_t led_pin_3,
    const uint8_t led_pin_4
)
{
    _initialize_led(_led_1_state, led_pin_1);
    _initialize_led(_led_2_state, led_pin_2);
    _initialize_led(_led_3_state, led_pin_3);
    _initialize_led(_led_4_state, led_pin_4);
}

void TankLED::loop()
{
    _current_millis = millis();
    _update_led(_led_1_state);
    _update_led(_led_2_state);
    _update_led(_led_3_state);
    _update_led(_led_4_state);
}

void TankLED::led_1_turn_on() { _turn_led_on(_led_1_state); }
void TankLED::led_1_turn_off() { _turn_led_off(_led_1_state); }

void TankLED::led_2_turn_on() { _turn_led_on(_led_2_state); }
void TankLED::led_2_turn_off() { _turn_led_off(_led_2_state); }

void TankLED::led_3_turn_on() { _turn_led_on(_led_3_state); }
void TankLED::led_3_turn_off() { _turn_led_off(_led_3_state); }

void TankLED::led_4_turn_on() { _turn_led_on(_led_4_state); }
void TankLED::led_4_turn_off() { _turn_led_off(_led_4_state); }

void TankLED::led_1_set_state(const uint16_t states[], const uint8_t state_count)
{
    _led_set_state(_led_1_state, states, state_count);
}

void TankLED::led_2_set_state(const uint16_t states[], const uint8_t state_count)
{
    _led_set_state(_led_2_state, states, state_count);
}

void TankLED::led_3_set_state(const uint16_t states[], const uint8_t state_count)
{
    _led_set_state(_led_3_state, states, state_count);
}

void TankLED::led_4_set_state(const uint16_t states[], const uint8_t state_count)
{
    _led_set_state(_led_4_state, states, state_count);
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
            // even states are on and grounding the pin turns the LED on
            digitalWrite(_led_state.led_pin, LOW);
        } else {
            // turn the LED off
            digitalWrite(_led_state.led_pin, HIGH);
        }
        _led_state.next_state_change = _current_millis + _led_state.states[_led_state.current_state];
    }
}
