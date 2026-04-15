#include "src/tank.h"

Tank tank;

// put global variables here

void setup()
{
  tank.initialize();
  // put your custom setup logic here
}

void loop()
{
  // this call must always be left in
  tank.loop();

  // here you can do logic or call functions
}
