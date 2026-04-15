#ifndef tank_led_h
#define tank_led_h

#include <Arduino.h>

#define LED_COUNT 3
#define MAX_BLINK_COUNT 10

enum LedState {
  on,
  off,
  blinking
};

struct Led {
    uint8_t led_pin;
    LedState state;
    uint8_t blink_count;
    uint16_t blinks[MAX_BLINK_COUNT];
    uint8_t current_blink_index;
    unsigned long next_blink_change;
};

class TankLed
{
    public:
        void setup(const uint8_t led_pins[]);
        void loop();
        void turn_on(uint8_t led_index);
        void turn_off(uint8_t led_index);
        void toggle(uint8_t led_index);
        void all_on();
        void all_off();
        void led_set_blinks(uint8_t led_index, const uint16_t blinks[], const uint8_t blink_count);

    private:
        void _initialize_led(struct Led & _led, const uint8_t led_pin);
        void _update_led(struct Led & _led);
        void _turn_led_on(struct Led & _led);
        void _turn_led_off(struct Led & _led);

        Led _leds[LED_COUNT];

        unsigned long _current_millis;
};

#endif
