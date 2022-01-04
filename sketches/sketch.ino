#include "tank.h"

Tank tank;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing ...");
    tank.setup();
    Serial.println("Tank initialized.");
}

void loop()
{
    tank.loop();
}
