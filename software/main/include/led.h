#pragma once
#include "Arduino.h"
#include "def_system.h"

class led {
  public:
    led();

    void setup();
    void blink();

  private:
    int led_pin;
};
