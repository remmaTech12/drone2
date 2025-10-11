#pragma once
#include "def_system.h"
#include "Bitcraze_PMW3901.h"
#include "lib/moving_average.h"

class flow_pmw3901 {
  public:
    flow_pmw3901();

    void setup();
    void read_motion_count();
    void calculate_delta_position(double height, float ang_data[3]);
    void get_delta_position_data(float data[2]);

  private:
    Bitcraze_PMW3901 sensor;

    int16_t pixel_dx_, pixel_dy_;
    MovingAverage pixel_dx_filter_;
    MovingAverage pixel_dy_filter_;

    float position_dx_ = 0.0;
    float position_dy_ = 0.0;
    float pre_roll_ang_ = 0.0;
    float pre_pitch_ang_ = 0.0;
};
