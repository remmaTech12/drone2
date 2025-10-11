#include "../include/control.h"

Control::Control() {}

void Control::setup() {
    set_pos_pid_gain();
}

void Control::set_pos_pid_gain() {
    // Initialize PID for position (x, y)
    pid_pos_.Kp = {300.0f, 300.0f};
    pid_pos_.Ki = {0.0f, 0.0f};
    pid_pos_.Kd = {0.0f, 0.0f};
    pid_pos_.max_err_i = 2.0f;

    pid_pos_.low_pass_filter = {LowPassFilter(), LowPassFilter()};
    pid_pos_.low_pass_filter[0].setup(0.1f);
    pid_pos_.low_pass_filter[1].setup(0.1f);

    pid_pos_.err_i = {0.0f, 0.0f};
    pid_pos_.pre_data = {0.0f, 0.0f};

    pid_pos_.out_data = {0.0f, 0.0f};
    pid_pos_.max_out_data = 5.0f;
}

void Control::calculate_xy_command(float ref_data[2], int cmd_data[4]) {
    constexpr float cmd_mid = 127;
    constexpr float dead_zone = 20;
    constexpr float cmd_min = cmd_mid - dead_zone;
    constexpr float cmd_max = cmd_mid + dead_zone;
    constexpr float cmd_gain = 100.0f;
    const int ver_cmd = cmd_data[2];  // vertical
    const int hor_cmd = cmd_data[3];  // horizontal

    if (cmd_min < ver_cmd && ver_cmd < cmd_max) ref_data[0] = 0.0f;
    else ref_data[0] = (float) +(ver_cmd - cmd_mid) / cmd_gain;

    if (cmd_min < hor_cmd && hor_cmd < cmd_max) ref_data[1] = 0.0f;
    else ref_data[1] = (float) -(hor_cmd - cmd_mid) / cmd_gain;
}

void Control::calculate_pid_pos(int cmd_data[4], float cur_data[2]) {
    float ref_data[2];
    calculate_xy_command(ref_data, cmd_data);

    float err_p[2];
    float filtered_err_d[2] = {0.0f, 0.0f};
    for (int i=0; i<2; i++) {
        // P
        err_p[i] = ref_data[i] - cur_data[i];
        // I
        pid_pos_.err_i[i] += err_p[i];
        limit_val(pid_pos_.err_i[i], -pid_pos_.max_err_i, pid_pos_.max_err_i);
        //if (std::abs(err_p[i]) < 0.05f) pid_pos_.err_i[i] = 0.0f;
        // D
        const float err_d = -(cur_data[i] - pid_pos_.pre_data[i]) / ((float)SAMPLING_POSITION_CONTROL_TIME_MS/1000.0f);
        filtered_err_d[i] = pid_pos_.low_pass_filter[i].filter(err_d);
        pid_pos_.pre_data[i] = cur_data[i];
    }

    for (int i=0; i<2; i++) {
        pid_pos_.out_data[i] = pid_pos_.Kp[i]*err_p[i]
                             + pid_pos_.Ki[i]*pid_pos_.err_i[i]
                             + pid_pos_.Kd[i]*filtered_err_d[i];
        //limit_val(pid_pos_.out_data[i], -pid_pos_.max_out_data, pid_pos_.max_out_data);
    }
    // tilt-cone limiter
    const float alpha = std::hypot(pid_pos_.out_data[0], pid_pos_.out_data[1]);
    const float alpha_max = 5.0;
    for (int i=0; i<2; i++) {
        float alpha_ratio = alpha < alpha_max ? 1.0f : alpha_max / alpha;
        pid_pos_.out_data[i] *= alpha_ratio;
    }
}

void Control::calculate_pid_ang(int cmd_data[4], float ang_data[3]) {
    float ref_data[3];
    float out_data[3] = {0.0f, 0.0f, 0.0f};
    ref_data[0] = -pid_pos_.out_data[1];
    ref_data[1] = +pid_pos_.out_data[0];

#ifdef DEBUG_ATTITUDE_CONTROL
    Serial.print("Position control command, roll: ");
    Serial.print(ref_data[0]);
    Serial.print(", pitch: ");
    Serial.println(ref_data[1]);
    Serial.print(", yaw: ");
    Serial.println(ref_data[2]);
#endif

    double yaw_input_in_arm_threshold = 200;
    if (cmd_data[0] > yaw_input_in_arm_threshold) ref_data[2] = 0;  // exclude the case for arm
    else ref_data[2] = (float) (cmd_data[1] - 127.0f) / 2.0f;

    calculate_pid(ref_data, ang_data, err_ang_data_i_, pre_ang_data_, pre_filtered_ang_dterm_data_, out_data,
                  Kp_ang_, Ki_ang_, Kd_ang_, SAMPLING_OUTER_TIME_MS);

    for (int i=0; i<3; i++) {
        constexpr float max_cmd_val = 255.0f;
        limit_val(out_data[i], -max_cmd_val, max_cmd_val);
        ang_ref_data_[i] = out_data[i];
    }

#ifdef DEBUG_ATTITUDE_CONTROL
    Serial.print("Attitude control command, roll: ");
    Serial.print(out_data[0]);
    Serial.print(", pitch: ");
    Serial.print(out_data[1]);
    Serial.print(", yaw: ");
    Serial.println(out_data[2]);
#endif
}

void Control::calculate_pid_angvel(float angvel_data[3]) {
    float out_data[3] = {0.0f, 0.0f, 0.0f};

    calculate_pid(ang_ref_data_, angvel_data, err_angvel_data_i_, pre_angvel_data_, pre_filtered_angvel_dterm_data_, out_data,
                  Kp_angvel_, Ki_angvel_, Kd_angvel_, SAMPLING_INNER_TIME_MS);

    float filtered_out_data[3] = {0.0f, 0.0f, 0.0f};
    double cutoff_freq = 10;
    low_pass_filter(cutoff_freq, pre_filtered_control_data_, out_data, filtered_out_data);

    for (int i=0; i<3; i++) {
        constexpr float max_cmd_val = 255.0f;
        limit_val(out_data[i], -max_cmd_val, max_cmd_val);
        angvel_ctl_data_[i] = out_data[i];
    }

#ifdef DEBUG_ANGVEL_CONTROL
    Serial.print("Angular velocity control command, roll: ");
    Serial.println(out_data[0]);
    Serial.print(", pitch: ");
    Serial.print(out_data[1]);
    Serial.print(", yaw: ");
    Serial.println(out_data[2]);
#endif
}

void Control::calculate_pid(float ref_data[3], float cur_data[3], float err_data_i[3],
                            float pre_data[3], float pre_filtered_dterm_data[3], float out_data[3],
                            float Kp[3], float Ki[3], float Kd[3], unsigned long sampling_time_ms) {
    float err_data_p[3];
    float data_d[3];

    for (int i=0; i<3; i++) {
        err_data_p[i]  = ref_data[i] - cur_data[i];
        err_data_i[i] += err_data_p[i];
        data_d[i]      = - (cur_data[i] - pre_data[i]) / ((float)sampling_time_ms/1000.0f);

        constexpr float max_err_val = 60.0f;
        limit_val(err_data_i[i], -max_err_val, max_err_val);
        //if (std::abs(err_data_p[i]) < 1.0f) err_data_i[i] = 0.0f;
        pre_data[i] = cur_data[i];
    }

    float filtered_data_d[3] = {0.0f, 0.0f, 0.0f};
    float cutoff_freq = 1.0f;
    low_pass_filter(cutoff_freq, pre_filtered_dterm_data, data_d, filtered_data_d);

    for (int i=0; i<3; i++) {
        out_data[i] = Kp[i]*err_data_p[i] + Ki[i]*err_data_i[i] + Kd[i]*filtered_data_d[i];
    }
}

void Control::limit_val(float &val, float min, float max) {
    if (val > max) { val = max; }
    if (val < min) { val = min; }
}

void Control::low_pass_filter(float cutoff_freq, float pre_filtered_data[3], float cur_data[3], float filtered_data[3]) {
    constexpr float kpre = 0.1;

    for (int i=0; i<3; i++) {
        filtered_data[i] = kpre*pre_filtered_data[i] + (1.0f - kpre)*cur_data[i];
        pre_filtered_data[i] = filtered_data[i];
    }
}

void Control::get_control_val(float ctl_data[3]) {
    ctl_data[0] = angvel_ctl_data_[0];
    ctl_data[1] = angvel_ctl_data_[1];
    ctl_data[2] = angvel_ctl_data_[2];
}