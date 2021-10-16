#include <Wire.h>
#include <PVision.h>

#include "tank.h"

unsigned long last_update_millis = 0;

Tank tank;

PVision ircam;

struct target {
    uint16_t x;
    uint16_t y;
    uint16_t size;
};

bool find_closest_target(struct target & t)
{
    byte result = ircam.read();
    uint8_t closest = 0;

    if (result & BLOB1) {
        t.x = ircam.Blob1.X;
        t.y = ircam.Blob1.Y;
        t.size = ircam.Blob1.Size;
        closest = t.size;
    }

    if (result & BLOB2 && ircam.Blob2.Size > closest) {
        t.x = ircam.Blob2.X;
        t.y = ircam.Blob2.Y;
        t.size = ircam.Blob2.Size;
        closest = t.size;
    }

    if (result & BLOB3 && ircam.Blob3.Size > closest) {
        t.x = ircam.Blob3.X;
        t.y = ircam.Blob3.Y;
        t.size = ircam.Blob3.Size;
        closest = t.size;
    }

    if (result & BLOB3 && ircam.Blob3.Size > closest) {
        t.x = ircam.Blob3.X;
        t.y = ircam.Blob3.Y;
        t.size = ircam.Blob3.Size;
        closest = t.size;
    }

    if (closest > 0) {
        return true;
    } else {
        return false;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing ...");
    tank.setup();
    Serial.println("Tank initialized.");
    //ircam.init();
    //Serial.println("Camera initialized.");
    tank.enable_motors(1);
    randomSeed(analogRead(0));
}

void loop()
{
    unsigned long current_millis = millis();

    //if (current_millis > last_update_millis + 500) {
    //    struct target my_target;
    //    if (find_closest_target(my_target)) {
    //        Serial.println("Target Detected");
    //        Serial.print(" X: ");
    //        Serial.print(my_target.x);
    //        Serial.print(" Y: ");
    //        Serial.print(my_target.y);
    //        Serial.print(" Size: ");
    //        Serial.println(my_target.size);
    //    }

    //    last_update_millis = current_millis;
    //}
    tank.loop();
}
