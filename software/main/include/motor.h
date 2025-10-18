#pragma once
#include "arm.h"
#include "Arduino.h"
#include "def_system.h"
#include "lib/low_pass_filter.h"

class Motor {
   public:
    Motor();

    void setup();
    void test_control(int cmd_val, Arm &arm);
    void control(int cmd_data[4], float pid_rpy[3], Arm &arm, float height);
    void stop_motor();

   private:
    void limit_command(int &cmd, int min, int max);
    int calculate_thrust(double thrust_scale, int cmd_data[4]);
    int calculate_thrust_based_on_height(int cmd_data[4], float height, double thrust_scale);
    void calculate_motor_control(float ctl_data[3], int motor_data[4], int cmd_thrust);

    double err_height_i_ = 0.0;
    std::vector<LowPassFilter> low_pass_filter_;
};