#include <PinChangeInterrupt.h>

#include "constants.h"
#include "mini_led.h"

#define IR_RECEIVE_PIN IR_RX_PIN
#define USE_CALLBACK_FOR_TINY_RECEIVER
// IMPORTANT: The above IR pre-processor directives must be defined before including TinyIRReceiver.hpp
#include "TinyIRReceiver.hpp"
#include "TinyIRSender.hpp"

int blink_interval = 2000;
uint32_t last_blink = 0;
boolean blink_state = false;

boolean ir_command_received = false;
uint16_t ir_command = 0;
boolean button_interrupt_flag = false;
MiniLed mini_led;

void button_interrupt() {
  button_interrupt_flag = true;
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));

  uint8_t pins[] = {LED_PIN};
  mini_led.setup(pins, HIGH);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON_PIN), button_interrupt, RISING);

  if (!initPCIInterruptForTinyReceiver()) {
    Serial.println(F("could not initialize IR"));
  }

  Serial.println(F("Initialized"));


  mini_led.on(0);
  delay(500);
  mini_led.off(0);
}

void loop()
{
  mini_led.loop();

  // Check if button is pressed
  if (button_interrupt_flag) {
    Serial.println("Button pressed");
    button_interrupt_flag = false;
    fire();
    mini_led.set_blinks(0, (const uint16_t[]){500, 500}, 2, 2);
  }

  if (ir_command_received) {
    Serial.print("Received IR: ");
    Serial.println(ir_command);
    ir_command_received = false;

    if (ir_command == IR_CODE_ASTERISK) {
      Serial.println("received hit");
      mini_led.set_blinks(0, (const uint16_t[]){500, 500}, 2, 6);
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
