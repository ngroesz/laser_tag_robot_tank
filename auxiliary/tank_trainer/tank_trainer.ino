#include <Debounce.h>


#include "constants.h"

#define IR_RECEIVE_PIN IR_RX_PIN
#define USE_CALLBACK_FOR_TINY_RECEIVER
// IMPORTANT: The above IR pre-processor directives must be defined before including TinyIRReceiver.hpp
#include "TinyIRReceiver.hpp"

const uint8_t INTERVAL_DEBOUNCE = 1;       // Update button state every 1ms
uint32_t timeDebounce = 0;                       // Time of last debounce update

int blink_interval = 2000;
uint32_t last_blink = 0;
boolean blink_state = false;

Debounce myButton(BUTTON_PIN, LOW);

boolean ir_command_received = false;
uint16_t ir_command = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);
  if (!initPCIInterruptForTinyReceiver()) {
    Serial.println(F("could not initialize IR"));
  }
  Serial.println(F("Initialized"));
}

void loop()
{
  if (millis() - timeDebounce >= INTERVAL_DEBOUNCE) {
    timeDebounce = millis();
    myButton.update(); // Update button state every INTERVAL_DEBOUNCE
  }

  // Check if button is pressed
  if (myButton.isPressed()) {
    Serial.println("Button pressed");
  }

  if (ir_command_received) {
    Serial.print("Received IR: ");
    Serial.println(ir_command);
    ir_command_received = false;
  }
}

void handleReceivedTinyIRData() {
  if (TinyIRReceiverData.Flags != IRDATA_FLAGS_IS_REPEAT && TinyIRReceiverData.Flags != IRDATA_FLAGS_PARITY_FAILED) {
    ir_command_received = true;
    ir_command = TinyIRReceiverData.Command;
  }
}
