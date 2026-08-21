#include <Debounce.h>


#include "constants.h"
#include "tank_led.h"

#define IR_RECEIVE_PIN IR_RX_PIN
#define USE_CALLBACK_FOR_TINY_RECEIVER
// IMPORTANT: The above IR pre-processor directives must be defined before including TinyIRReceiver.hpp
#include "TinyIRReceiver.hpp"
#include "TinyIRSender.hpp"

// debounce code copied from Debounce library example code
const uint8_t debounce_delay = 1;
uint32_t time_debounce = 0;

int blink_interval = 2000;
uint32_t last_blink = 0;
boolean blink_state = false;

Debounce fire_button(BUTTON_PIN, LOW);

boolean ir_command_received = false;
uint16_t ir_command = 0;

TankLed tank_led;

void setup()
{
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));

  uint8_t pins[] = {LED_PIN, 13};
  tank_led.setup(pins, HIGH);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!initPCIInterruptForTinyReceiver()) {
    Serial.println(F("could not initialize IR"));
  }

  Serial.println(F("Initialized"));
}

void loop()
{
  tank_led.loop();
  if (millis() - time_debounce >= debounce_delay) {
    time_debounce = millis();
    fire_button.update(); // Update button state every debounce_delay
  }

  // Check if button is pressed
  if (fire_button.isPressed()) {
    Serial.println("Button pressed");
    fire();
    tank_led.set_blinks(1, (const uint16_t[]){500, 500}, 2, 6);
  }

  if (ir_command_received) {
    Serial.print("Received IR: ");
    Serial.println(ir_command);
    ir_command_received = false;

    if (ir_command == IR_CODE_ASTERISK) {
      Serial.println("received hit");
      tank_led.set_blinks(0, (const uint16_t[]){500, 500}, 2, 6);
    }
  }
}

void fire()
{
  sendNEC(IR_TX_PIN, 0x0, IR_CODE_ASTERISK, 0);
}

void handleReceivedTinyIRData() {
  if (TinyIRReceiverData.Flags != IRDATA_FLAGS_IS_REPEAT && TinyIRReceiverData.Flags != IRDATA_FLAGS_PARITY_FAILED) {
    ir_command_received = true;
    ir_command = TinyIRReceiverData.Command;
  }
}
