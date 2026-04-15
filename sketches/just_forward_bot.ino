#include "src/tank.h"

Tank tank;

uint8_t status;

void setup()
{
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\n"));

    Serial.println("Initializing ...");
    tank.initialize();
    Serial.println("Tank initialized.");
    tank.drive_forward();
}

void loop()
{
}
