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

uint8_t pins[] = {LED_PIN};
MiniLed mini_led(pins, 1, LOW);

void button_interrupt() {
  button_interrupt_flag = true;
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));

 // uint8_t pins[] = {LED_PIN};
 // mini_led.setup(pins, 1, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON_PIN), button_interrupt, RISING);

  if (!initPCIInterruptForTinyReceiver()) {
    Serial.println(F("could not initialize IR"));
  }

  Serial.println(F("Initialized"));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //Serial.println("on");
  //mini_led.on(0);
  //delay(5000);
  //mini_led.off(0);
  //Serial.println("off");
}

void loop()
{
  //mini_led.loop();

  // Check if button is pressed
  if (button_interrupt_flag) {
    Serial.println("Button pressed");
    button_interrupt_flag = false;
    //fire();
    // blink once
    Serial.println("on");
    mini_led.on(0);
    //pinMode(LED_PIN, OUTPUT);
    //digitalWrite(LED_PIN, LOW);
    //mini_led.blink(0, 1);
  }

  if (ir_command_received) {
    Serial.print("Received IR: ");
    Serial.println(ir_command);
    ir_command_received = false;

    if (ir_command == IR_CODE_ASTERISK) {
      Serial.println("off");
      mini_led.off(0);
      //pinMode(LED_PIN, OUTPUT);
      //digitalWrite(LED_PIN, HIGH);
      // blink three times
     // mini_led.blink(0, 3);
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
