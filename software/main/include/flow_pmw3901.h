#pragma once
#include "def_system.h"
#include "Bitcraze_PMW3901.h"

class flow_pmw3901 {
  public:
    flow_pmw3901();

    void setup();
    void readMotionCount(int data[2]);
    void printMotionCount();

  private:
    Bitcraze_PMW3901 sensor;
    int16_t dx, dy;
};
