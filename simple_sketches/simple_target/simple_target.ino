// A bare bones target sketch
// If receive the IR code for being shot, light an LED indicator

// needs to be defined before TinyIRReceiver is included
#define USE_CALLBACK_FOR_TINY_RECEIVER
#define NO_LED_FEEDBACK_CODE
#include <Arduino.h>
#include "misc.h"
#include "TinyIRReceiver.hpp"
#include "ir_codes.h"

#define IR_RECEIVE_PIN 2
#define LED_SHOT_INDICATOR A0
#define SHOT_INDICATOR_MILLIS 1000

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial to become available. Is optimized away for some cores.

    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing TinyIR library version " VERSION_TINYIR));

    // Enables the interrupt generation on change of IR input signal
    if (!initPCIInterruptForTinyReceiver()) {
        Serial.println(F("No interrupt available for pin " STR(IR_RECEIVE_PIN))); // optimized out by the compiler, if not required :-)
    }

    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_SHOT_INDICATOR, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_SHOT_INDICATOR, OUTPUT);
}

bool handle_ir_command = 0;
uint8_t ir_command;
// we want to probably do a struct or something for easier readability
uint8_t shot_state = 0;
unsigned long shot_state_millis;

void loop() {
    if (handle_ir_command) {
        handle_ir_command = 0;
        Serial.print("handle IR command ");
        Serial.println(ir_command);
        if (ir_command == IR_CODE_SELECT) {
            shot_state = 1;
        }
    }

    state_machine();
}

void state_machine() {
    if (shot_state == 1) {
        digitalWrite(LED_SHOT_INDICATOR, HIGH);
        Serial.println("I been SHOT");
        shot_state_millis = millis();
        shot_state = 2;
    } else if(shot_state == 2) {
        if (shot_state_millis + SHOT_INDICATOR_MILLIS < millis()) {
            digitalWrite(LED_SHOT_INDICATOR, LOW);
            shot_state = 0;
        }
    }
}

void handleReceivedTinyIRData() {
    // because this happens in an interrupt context, we want to quickly set flags and finish
    // the actual logic can be handled elsewhere
    handle_ir_command = 1;
    ir_command = TinyIRReceiverData.Command;
}
