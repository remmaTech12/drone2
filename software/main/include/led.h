#pragma once
#include "Arduino.h"
#include "def_system.h"

class led {
  public:
    led();

    void setup();
    void blink();
    void on();
    void off();

  private:
    int led_pin;
};
