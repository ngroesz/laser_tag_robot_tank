#include "Arduino.h"
#include "tank_led.h"

void TankLED::setup(uint8_t led_pin_1, uint8_t led_pin_2, uint8_t led_pin_3, uint8_t led_pin_4)
{
    _led_1_state.led_pin = led_pin_1;
    _led_1_state.led_is_on_or_off = true;
    _led_1_state.state_count = 0;

    pinMode(_led_1_state.led_pin, OUTPUT);
    //pinMode(LED_PIN_2, OUTPUT);
    //pinMode(LED_PIN_3, OUTPUT);
    //pinMode(LED_PIN_4, OUTPUT);

    digitalWrite(_led_1_state.led_pin, HIGH);
    //digitalWrite(LED_PIN_2, HIGH);
    //digitalWrite(LED_PIN_3, HIGH);
    //digitalWrite(LED_PIN_4, HIGH);
}

void TankLED::loop()
{
    _update_led(_led_1_state);
}

void TankLED::led_1_turn_on()
{
    _turn_led_on(_led_1_state);
}

void TankLED::led_1_turn_off()
{
    _turn_led_off(_led_1_state);
}

void TankLED::led_1_set_state(const uint16_t states[], const uint8_t state_count)
{
    _led_set_state(_led_1_state, states, state_count);
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
    _led_state.current_state = 0;
    _led_state.next_state_change = 0;
}

void TankLED::_update_led(struct led_state & _led_state)
{
    if (_led_state.led_is_on_or_off)  {
        return;
    }

    _current_millis = millis();
    if (_current_millis > _led_state.next_state_change) {
        Serial.println("state change!");
        ++_led_state.current_state;
        if (_led_state.current_state == _led_state.state_count) {
            _led_state.current_state = 1;
        }
        if(_led_state.current_state % 2 == 0) {
            Serial.print("turning on: ");
            // even states are on and grounding the pin turns the LED on
            digitalWrite(_led_state.led_pin, LOW);
        } else {
            Serial.print("turning off: ");
            // turn the LED off
            digitalWrite(_led_state.led_pin, HIGH);
        }
        Serial.println(_led_state.current_state);
        _led_state.next_state_change = _current_millis + _led_state.states[_led_state.current_state - 1];
        Serial.print("staet count: ");
        Serial.println(_led_state.state_count);
        Serial.print("now: ");
        Serial.println(_current_millis);
        Serial.print("next state change: ");
        Serial.println(_led_state.next_state_change);
    } else {
        Serial.println("no state change");
    }
}
