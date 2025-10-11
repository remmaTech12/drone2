#pragma once
#include "def_system.h"
#include "lib/low_pass_filter.h"
#include "Arduino.h"

// struct for gain
struct PID {
    // parameter
    std::vector<float> Kp;
    std::vector<float> Ki;
    std::vector<float> Kd;
    float max_err_i;
    unsigned long sampling_time_ms;

    // filter
    std::vector<LowPassFilter> d_filter;

    // state
    std::vector<float> err_i;
    std::vector<float> pre_data;

    // input
    std::vector<float> ref_data;
    std::vector<float> cur_data;

    // output
    std::vector<float> out_data;
    float max_out_data;
};

class Control {
   public:
    Control();

    void setup();
    void calculate_pid_pos(int cmd_data[4], float cur_data[2]);
    void calculate_pid_ang(int cmd_data[4], float ang_data[3]);
    void calculate_pid_angvel(float angvel_data[3]);
    void get_control_val(float ctl_data[3]);

   private:
    std::vector<float> calculate_joystick_to_xy_command(int cmd_data[4]);
    std::vector<float> calculate_xy_command_to_ang_command(int cmd_data[4]);
    void calculate_pid(PID &pid);
    void limit_val(float &val, float min, float max);
    void set_pos_pid();
    void set_ang_pid();
    void set_angvel_pid();
    void set_angvel_output_filter();

    // PID
    PID pid_pos_;
    PID pid_ang_;
    PID pid_angvel_;

    // filter
    std::vector<LowPassFilter> angvel_output_filter_;
};