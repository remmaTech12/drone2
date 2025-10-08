#pragma once
#include "def_system.h"
#include "Bitcraze_PMW3901.h"
#include "lib/moving_average.hpp"

class flow_pmw3901 {
  public:
    flow_pmw3901();

    void setup();
    void readMotionCount(int data[2]);
    void printMotionCount();
    void calculate_velocity_position(double height);
    void get_position_data(float data[2]);

  private:
    Bitcraze_PMW3901 sensor;
    int16_t dx, dy;
    MovingAverage dx_filter;
    MovingAverage dy_filter;
    unsigned long previous_ms = 0;
    float vx = 0.0;
    float vy = 0.0;
    float x = 0.0;
    float y = 0.0;
};
