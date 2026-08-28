#include "mini_tone.h"

void MiniTone::setup(uint8_t speaker_pin) {
  this->speaker_pin = speaker_pin;
  pinMode(speaker_pin, OUTPUT);
  currently_playing = false;
}

void MiniTone::play(const uint16_t* tones, const size_t tones_size) {
  this->tones = tones;
  this->tones_size = tones_size;
  current_tone_index = 0;
  currently_playing = true;
  uint16_t duration = tones[current_tone_index + 1];
  // instead of waiting for loop(), we will start playing the tone right now
  tone(speaker_pin, tones[current_tone_index], duration);
  next_tone_millis = millis() + duration + PAUSE_BETWEEN_TONES;
}

void MiniTone::loop() {
  if (!currently_playing) {
    return;
  }

  if (next_tone_millis <= millis()) {
    Serial.println("tone change");
    current_tone_index += 2;
    if (current_tone_index + 1 > tones_size) {
      currently_playing = false;
      return;
    }
    uint16_t duration = tones[current_tone_index + 1];
    tone(speaker_pin, tones[current_tone_index], duration);
    next_tone_millis = millis() + duration + PAUSE_BETWEEN_TONES;
  }
}
