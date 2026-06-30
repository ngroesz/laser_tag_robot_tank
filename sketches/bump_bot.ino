/* bump_bot.ino
* A bot that just drives around and bumps into obstacles
*/

#include "src/tank.h"

Tank tank;

uint8_t status;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing ...");
    tank.initialize();
    Serial.println("Tank initialized.");
    tank.set_bump_front_callback(bump_front);
    status = 0;
}

void loop()
{

  switch (status) {
    // initial status, tank is stopped
    case 0:
        tank.drive_forward();
        status = 1;
        break;
  }
  tank.loop();

}

void bump_front() {
  Serial.println("bump front!");
  status = 2;
  tank.drive_reverse_target(10, reverse_target_reached);
}

void reverse_target_reached() {
  Serial.println("done reversing");
  status = 3;
  tank.drive_stop();
}
