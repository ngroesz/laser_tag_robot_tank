#ifndef tank_led_h
#define tank_led_h

#include <Arduino.h>

struct led_state {
    uint8_t led_pin;
    boolean led_is_on_or_off;
    uint8_t state_count;
    uint16_t states[10];
    uint8_t current_state;
    unsigned long next_state_change;
};

class TankLED
{
    public:
        void setup(const uint8_t led_pin_1, const uint8_t led_pin_2, const uint8_t led_pin_3, const uint8_t led_pin_4);
        void loop();
        void led_1_turn_on();
        void led_1_turn_off();
        void led_2_turn_on();
        void led_2_turn_off();
        void led_3_turn_on();
        void led_3_turn_off();
        void led_4_turn_on();
        void led_4_turn_off();
        void led_1_set_state(const uint16_t states[], const uint8_t state_count);
        void led_2_set_state(const uint16_t states[], const uint8_t state_count);
        void led_3_set_state(const uint16_t states[], const uint8_t state_count);
        void led_4_set_state(const uint16_t states[], const uint8_t state_count);

    private:
        void _initialize_led(struct led_state & _led_state, const uint8_t led_pin);
        void _led_set_state(struct led_state & _led_state, const uint16_t states[], const uint8_t state_count);
        void _update_led(struct led_state & _led_state);
        void _turn_led_on(struct led_state & _led_state);
        void _turn_led_off(struct led_state & _led_state);

        struct led_state _led_1_state;
        struct led_state _led_2_state;
        struct led_state _led_3_state;
        struct led_state _led_4_state;

        unsigned long _current_millis;
};

#endif
