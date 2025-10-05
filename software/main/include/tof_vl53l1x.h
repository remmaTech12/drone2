#pragma once
#include "def_system.h"
#include <VL53L1X.h>

class tof_vl53l1x {
  public:
    tof_vl53l1x();

    void setup();
    void readDistance(int16_t &dist);
    int16_t getDistance();
    void printDistance();

  private:
    VL53L1X sensor;
    int16_t distance;
};
