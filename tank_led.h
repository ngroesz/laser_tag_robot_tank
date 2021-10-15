#ifndef tank_led_h
#define tank_led_h

#include <Arduino.h>
#define LED_COUNT 4
#define MAX_STATE_COUNT 10

struct led_state {
    uint8_t led_pin;
    boolean led_is_on_or_off;
    uint8_t state_count;
    uint16_t states[MAX_STATE_COUNT];
    uint8_t current_state;
    unsigned long next_state_change;
};

class TankLED
{
    public:
        void setup(const uint8_t led_pins[]);
        void loop();
        void led_turn_on(uint8_t led_index);
        void led_turn_off(uint8_t led_index);
        void all_on();
        void all_off();
        void led_set_state(uint8_t led_index, const uint16_t states[], const uint8_t state_count);

    private:
        void _initialize_led(struct led_state & _led_state, const uint8_t led_pin);
        void _led_set_state(struct led_state & _led_state, const uint16_t states[], const uint8_t state_count);
        void _update_led(struct led_state & _led_state);
        void _turn_led_on(struct led_state & _led_state);
        void _turn_led_off(struct led_state & _led_state);

        struct led_state _led_states[LED_COUNT];

        unsigned long _current_millis;
};

#endif
