#ifndef tank_led_h
#define tank_led_h

#include <Arduino.h>

#define BLINK_MILLIS 500
#define MAX_LED_COUNT 3

enum LedState {
  led_on,
  led_off,
  led_blinking
};

struct Led {
  uint8_t led_pin;
  LedState state;
  uint8_t blink_count;
  uint8_t target_blinks;
  uint32_t last_change_millis;
};

class MiniLed
{
  public:
    // TODO: consider to move setup to a constructor
    MiniLed(const uint8_t led_pins[], const uint8_t led_count, const boolean on_state = HIGH);
    void loop();
    void on(const uint8_t led_index);
    void off(const uint8_t led_index);
    void blink(const uint8_t led_index, const uint8_t blink_count = 0);
    void toggle(uint8_t led_index);
    void all_on();
    void all_off();
    //void set_blinks(uint8_t led_index, const uint16_t blinks[], const uint8_t blinks_size);
    //void set_blinks(uint8_t led_index, const uint16_t blinks[], const uint8_t blinks_size, const uint8_t max_blinks);

  private:
    void _initialize_led(struct Led & _led, const uint8_t led_pin);
    void _reset_led(struct Led & _led);
    void _update_led(struct Led & _led);
    //void _led_on(struct Led & _led);
    //void _led_off(struct Led & _led);

    void _print_led_state(struct Led & _led);

    boolean _on_state;
    boolean _off_state;

    // TODO: can this be made dynamic
    Led _leds[MAX_LED_COUNT];

    uint8_t _led_count;  
    unsigned long _current_millis;
};

#endif
