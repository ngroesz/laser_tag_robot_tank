/*
    This code is adapted from a stellar tutorial found here at https://learn.adafruit.com/ir-sensor/ir-remote-signals
*/

#define IR_CODE_POWER 0x10EFD827
#define IR_CODE_A 0x10EFF807
#define IR_CODE_B 0x10EF7887
#define IR_CODE_C 0x10EF58A7
#define IR_CODE_UP 0x10EFA05F
#define IR_CODE_DOWN 0x10EF00FF
#define IR_CODE_LEFT 0x10EF10EF
#define IR_CODE_RIGHT 0x10EF807F
#define IR_CODE_SELECT 0x10EF20DF

#define IR_RECEIVER_PORT PIND
#define IR_MAXPULSE 40000
#define IR_RESOLUTION 20
#define IR_BEGIN_MIN_LENGTH 200
#define IR_END_MIN_LENGTH 1600
#define IR_DATA_MIN_LENGTH 1200
#define IR_HIGH_PULSE_LENGTH 50
#define IR_DELAY_BETWEEN_TRANSMISSIONS_MILLIS 100
#define IR_MINIMUM_CODE_VALUE 284098560
#define IR_MAXIMUM_CODE_VALUE 284164095

uint16_t highpulse, lowpulse;
uint8_t current_pulse, current_bit;
unsigned long result;
boolean ir_data_begun, finished;

unsigned long read_ir_data(int ir_receiver_pin)
{
    result = 0;
    current_pulse = 0;
    ir_data_begun = finished = 0;

    Serial.println("read ir data");
    //while(!finished) {
    //    Serial.println("pulse");
    //    highpulse = lowpulse = 0;
    //    while (IR_RECEIVER_PORT & (1 << ir_receiver_pin)) {
    //        highpulse++;
    //        delayMicroseconds(IR_RESOLUTION);

    //        if (highpulse >= IR_MAXPULSE && current_pulse != 0) {
    //            return 0;    
    //        }
    //    }
    //    if (!ir_data_begun) {
    //        if(highpulse > IR_BEGIN_MIN_LENGTH) {
    //            ir_data_begun = 1;
    //        }
    //    } else {
    //        if (highpulse > IR_END_MIN_LENGTH) {
    //            finished = 1;
    //        } else {
    //            if(highpulse > IR_HIGH_PULSE_LENGTH) {
    //                current_bit = 1;
    //            } else {
    //                current_bit = 0;
    //            }
    //            result = result << 1 | current_bit;
    //        }
    //    }

    //    while (!(IR_RECEIVER_PORT & _BV(ir_receiver_pin))) {
    //        lowpulse++;
    //        delayMicroseconds(IR_RESOLUTION);
    //        if (lowpulse >= IR_MAXPULSE && current_pulse != 0) {
    //            return 0;
    //        }
    //    }
    //    current_pulse++;
    //}
    Serial.println("done");

    if (result >= IR_MINIMUM_CODE_VALUE && result <= IR_MAXIMUM_CODE_VALUE) {
        return result;
    } else {
        return 0;
    }
}
