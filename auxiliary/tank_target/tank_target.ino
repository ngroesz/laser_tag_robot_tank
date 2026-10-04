#include <PinChangeInterrupt.h>

#define BUTTON_PIN 2
#define IR_RX_PIN 3
#define IR_TX_PIN 15
#define LED_PIN 6
#define IR_CODE_ASTERISK 22
#define FIRE_IGNORE_MILLIS 1

#include "mini_led.h"

#define IR_RECEIVE_PIN IR_RX_PIN
#define USE_CALLBACK_FOR_TINY_RECEIVER
// IMPORTANT: The above IR pre-processor directives must be defined before including TinyIRReceiver.hpp
#include "TinyIRReceiver.hpp"
#include "TinyIRSender.hpp"

#define DEBUG_OUTPUT

boolean ir_command_received = false;
uint16_t ir_command = 0;
boolean button_interrupt_flag = false;

uint32_t last_fire_millis = 0;

MiniLed mini_led;

void button_interrupt() {
  button_interrupt_flag = true;
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));

  uint8_t pins[] = {LED_PIN};
  mini_led.setup(pins, 1, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON_PIN), button_interrupt, RISING);

  if (!initPCIInterruptForTinyReceiver()) {
    Serial.println(F("could not initialize IR"));
  }

  Serial.println(F("Initialized"));

  mini_led.blink(0, 2);
}

void loop()
{
  mini_led.loop();

  // Check if button is pressed
  if (button_interrupt_flag) {
#ifdef DEBUG_OUTPUT
    Serial.println("Button pressed");
#endif
    button_interrupt_flag = false;
    fire();

    mini_led.blink(0, 1);
  }

  if (ir_command_received) {
#ifdef DEBUG_OUTPUT
    Serial.print("Received IR: ");
    Serial.println(ir_command);
#endif
    ir_command_received = false;

    // without a delay, reflected light can register as a self-hit
    // a delay of as little as a millisecond turns out to be enought to avoid this
    if (ir_command == IR_CODE_ASTERISK && millis() > last_fire_millis + 1) {
#ifdef DEBUG_OUTPUT
      Serial.println("Received hit ");
#endif
      mini_led.blink(0, 3);
    }
  }
}

void fire()
{
  sendNEC(IR_TX_PIN, 0x0, IR_CODE_ASTERISK, 0);
  last_fire_millis = millis();
}

void handleReceivedTinyIRData() {
  if (TinyIRReceiverData.Flags != IRDATA_FLAGS_IS_REPEAT && TinyIRReceiverData.Flags != IRDATA_FLAGS_PARITY_FAILED) {
    ir_command_received = true;
    ir_command = TinyIRReceiverData.Command;
  }
}
