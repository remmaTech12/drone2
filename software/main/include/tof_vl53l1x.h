#pragma once
#include "def_system.h"
#include <VL53L1X.h>

class tof_vl53l1x {
  public:
    tof_vl53l1x();

    void setup();
    void read_distance();
    float get_distance();
    float get_height(float ang_data[3]);

  private:
    VL53L1X sensor_;
    float distance_;
};
