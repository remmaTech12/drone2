#pragma once
#include "def_system.h"
#include "arm.h"
#include <Adafruit_BNO055.h>

class imu_bno055 {
  public:
    imu_bno055();

    void setup();
    void get_attitude_data(float data[3]);
    void get_angvel_data(float data[3]);
    void get_accel_data(float data[3]);
    void emergency_stop(Arm &arm);

  private:
    template<typename T> bool is_nan_or_inf(const std::vector<T> &arr);
    void calibrate();
    float clamp_deg(float value);

    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
    bool initialized_ = false;
    std::vector<float> initial_attitude_data_ = {0.0f, 0.0f, 0.0f};
    std::vector<float> attitude_data_ = {0.0f, 0.0f, 0.0f};
    std::vector<float> angvel_data_ = {0.0f, 0.0f, 0.0f};
};
