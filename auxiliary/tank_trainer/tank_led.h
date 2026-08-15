#ifndef tank_led_h
#define tank_led_h

#include <Arduino.h>

#define LED_COUNT 3
#define MAX_BLINKS_SIZE 10


enum LedState {
  led_on,
  led_off,
  led_blinking
};

struct Led {
  uint8_t led_pin;
  LedState state;
  uint8_t blink_count;
  uint8_t max_blinks;
  uint8_t blinks_size;
  uint16_t blinks[MAX_BLINKS_SIZE];
  uint8_t current_blink_index;
  unsigned long next_blink_change;
};

class TankLed
{
    public:
        // TODO: consider to move setup to a constructor
        void setup(const uint8_t led_pins[], boolean on_state = HIGH);
        void loop();
        void on(uint8_t led_index);
        void off(uint8_t led_index);
        void toggle(uint8_t led_index);
        void all_on();
        void all_off();
        void set_blinks(uint8_t led_index, const uint16_t blinks[], const uint8_t blinks_size);
        void set_blinks(uint8_t led_index, const uint16_t blinks[], const uint8_t blinks_size, const uint8_t max_blinks);

    private:
        void _initialize_led(struct Led & _led, const uint8_t led_pin);
        void _reset_led(struct Led & _led);
        void _update_led(struct Led & _led);
        void _led_on(struct Led & _led);
        void _led_off(struct Led & _led);

        void _print_led_state(struct Led & _led);
        
        boolean _on_state;
        boolean _off_state;

        Led _leds[LED_COUNT];

        unsigned long _current_millis;
};

#endif
