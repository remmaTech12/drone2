#include "../include/motor.h"

Motor::Motor() {}

void Motor::setup() {
    pinMode(MOTOR_PWM1, OUTPUT);
    pinMode(MOTOR_PWM2, OUTPUT);
//    pinMode(MOTOR_PWM3, OUTPUT);
//    pinMode(MOTOR_PWM4, OUTPUT);
    pinMode(MOTOR_PWM5, OUTPUT);
    pinMode(MOTOR_PWM6, OUTPUT);

    analogWrite(MOTOR_PWM1, 0);
    analogWrite(MOTOR_PWM2, 0);
//    analogWrite(MOTOR_PWM3, 0);
//    analogWrite(MOTOR_PWM4, 0);
    analogWrite(MOTOR_PWM5, 0);
    analogWrite(MOTOR_PWM6, 0);

    low_pass_filter_.resize(4);
    for (int i = 0; i < 4; i++) {
        low_pass_filter_[i].setup(0.3);
    }

    prev_time_ = millis();
}

void Motor::test_control(int motor_val) {
    constexpr int offset = 118;
    int test_motor_val = (motor_val - offset) * 2.0;
    limit_command(test_motor_val, 0, 255);

    analogWrite(MOTOR_PWM6, test_motor_val);
    analogWrite(MOTOR_PWM5, test_motor_val);
    analogWrite(MOTOR_PWM2, test_motor_val);
    analogWrite(MOTOR_PWM1, test_motor_val);
}

void Motor::limit_command(int &cmd, int min, int max) {
    if (cmd > max) { cmd = max; }
    if (cmd < min) { cmd = min; }
}

void Motor::stop_motor() {
    analogWrite(MOTOR_PWM1, 0);
    analogWrite(MOTOR_PWM2, 0);
    analogWrite(MOTOR_PWM3, 0);
    analogWrite(MOTOR_PWM4, 0);
    analogWrite(MOTOR_PWM5, 0);
    analogWrite(MOTOR_PWM6, 0);
}

/*
void Motor::debug_print(int data[4]) {
    Serial.print("thrust: ");
    Serial.print(data[0]);
    Serial.print(", roll: ");
    Serial.print(data[3]);
    Serial.print(", pitch: ");
    Serial.print(data[2]);
    Serial.print(", yaw: ");
    Serial.println(data[1]);
}

void Motor::format_cmd_data(int cmd_data[4]) {
    int control_damper = 3;
    int cmd_thrust = cmd_data[0];
    int cmd_roll   = (cmd_data[3] - 127)/control_damper;
    int cmd_pitch  = (cmd_data[2] - 127)/control_damper;
    int cmd_yaw    = (cmd_data[1] - 127)/control_damper;

    m_recv_cmd[0] = + cmd_roll - cmd_pitch - cmd_yaw + cmd_thrust;
    m_recv_cmd[1] = + cmd_roll + cmd_pitch + cmd_yaw + cmd_thrust;
    m_recv_cmd[2] = - cmd_roll + cmd_pitch - cmd_yaw + cmd_thrust;
    m_recv_cmd[3] = - cmd_roll - cmd_pitch + cmd_yaw + cmd_thrust;

    for (int i = 0; i < 4; i++) {
        limit_command(m_recv_cmd[i], 0, LIMIT_MOTOR);
    };
#ifdef DEBUG_RECV_COMMAND
    Serial.print("RECEIVE COMMAND: ");
    debug_print(m_recv_cmd);
#endif
}
*/

void Motor::control(int cmd_data[4], float ctl_data[3], Arm &arm, int16_t height) {
    if (arm.get_arm_status() == false) { 
        stop_motor();
        return;
    }

#ifdef DEBUG_MOTOR_COMMAND
    Serial.print("roll_ctrl: ");
    Serial.print(ctl_data[0]);
    Serial.print(", pitch_ctrl: ");
    Serial.print(ctl_data[1]);
    Serial.print(", yaw_ctrl: ");
    Serial.println(ctl_data[2]);
#endif

    int motor_data[4] = {0, 0, 0, 0};
    int cmd_thrust = 0;
    double thrust_scale = 0.6;

    cmd_thrust = calculate_thrust(thrust_scale, cmd_data);
    //cmd_thrust = calculate_thrust_based_on_height(cmd_data, height, thrust_scale);
    calculate_motor_control(ctl_data, motor_data);

    for (int i = 0; i < 4; i++) {
        double ctl_limit = LIMIT_MOTOR * (1.0f - thrust_scale);
        limit_command(motor_data[i], 0, ctl_limit);
        motor_data[i] += cmd_thrust;
        motor_data[i] = low_pass_filter_[i].filter(motor_data[i]);
        limit_command(motor_data[i], 0, LIMIT_MOTOR);
    };

    analogWrite(MOTOR_PWM6, motor_data[0]);
    analogWrite(MOTOR_PWM5, motor_data[1]);
    analogWrite(MOTOR_PWM2, motor_data[2]);
    analogWrite(MOTOR_PWM1, motor_data[3]);

#ifdef DEBUG_MOTOR_COMMAND
    Serial.print("MOTOR COMMAND: ");
    Serial.print("motor0: ");
    Serial.print(motor_data[0]);
    Serial.print(", motor1: ");
    Serial.print(motor_data[1]);
    Serial.print(", motor2: ");
    Serial.print(motor_data[2]);
    Serial.print(", motor3: ");
    Serial.println(motor_data[3]);
#endif
}

int Motor::calculate_thrust(double thrust_scale, int cmd_data[4]) {
    constexpr double kth = 40;
    constexpr double kth_scale = 3.0;
    if (cmd_data[0] < kth) {
        cmd_data[0] *= kth_scale;
    } else {
        cmd_data[0] = ((LIMIT_MOTOR - kth_scale*kth) / (LIMIT_MOTOR - kth)) * (cmd_data[0] - kth) + kth_scale*kth;
    }
    int cmd_thrust = cmd_data[0]*thrust_scale;
    limit_command(cmd_thrust, 0, LIMIT_MOTOR*thrust_scale);

    return cmd_thrust;
}

int Motor::calculate_thrust_based_on_height(int cmd_data[4], int16_t height, double thrust_scale) {
    // TODO: should be in control.cpp
    constexpr double max_height = 200.0;
    constexpr double max_cmd_thrust = 255.0;
    const double raw_cmd_thrust = cmd_data[0];
    const double target_height = max_height / max_cmd_thrust * raw_cmd_thrust;

    constexpr double Kp = 1.0;
    constexpr double Ki = 0.0;

    const double err_height = target_height - height;
    const double curr_time = millis();
    const double dt = (curr_time - prev_time_) / 1000.0;
    prev_time_ = curr_time;
    err_height_i_ += err_height * dt;
    err_height_i_ = std::clamp(err_height_i_, -500.0, 500.0);

    int cmd_thrust = Kp * err_height + Ki * err_height_i_;
    constexpr int offset_thrust = 50;
    constexpr int offset_thrust_threshold = 100;
    if (raw_cmd_thrust > offset_thrust_threshold) cmd_thrust += offset_thrust;
    limit_command(cmd_thrust, 0, LIMIT_MOTOR*thrust_scale);

    return cmd_thrust;
}

void Motor::calculate_motor_control(float ctl_data[3], int motor_data[4]) {
    double offset_motor[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    motor_data[0] = + ctl_data[0] - ctl_data[1] - ctl_data[2] + offset_motor[0];
    motor_data[1] = + ctl_data[0] + ctl_data[1] + ctl_data[2] + offset_motor[1];
    motor_data[2] = - ctl_data[0] + ctl_data[1] - ctl_data[2] + offset_motor[2];
    motor_data[3] = - ctl_data[0] - ctl_data[1] + ctl_data[2] + offset_motor[3];
}