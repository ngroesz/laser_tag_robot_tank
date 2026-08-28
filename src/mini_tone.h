#include <Arduino.h>

#define PAUSE_BETWEEN_TONES 500

class MiniTone {
  public:
    void setup(uint8_t speaker_pin);
    void play(const uint16_t* tones, const size_t tones_size);
    void loop();

  private:
    uint8_t speaker_pin;
    const uint16_t* tones;
    const uint16_t* tones_size;
    boolean currently_playing;
    uint16_t next_tone_millis;
    uint8_t current_tone_index;
};
