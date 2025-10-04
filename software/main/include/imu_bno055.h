#pragma once
#include "def_system.h"
#include <Adafruit_BNO055.h>
#include "MadgwickAHRS.h"

class imu_bno055 {
  public:
    imu_bno055();

    void setup();
    void get_attitude_data(float data[3]);
    void get_accel_data(float data[3]);
    void get_angvel_data(float data[3]);
    void calibrate();
    void printIMUEvent(sensors_event_t* event);

  private:
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
    Madgwick madgwick;
};
