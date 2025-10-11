#include "../include/imu_bno055.h"

imu_bno055::imu_bno055() {
}

void imu_bno055::setup() {
  while (!bno.begin(OPERATION_MODE_IMUPLUS))
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
}

float imu_bno055::clamp_deg(float value) {
  while (value >  180.0) value -= 360.0;
  while (value < -180.0) value += 360.0;

  return value;
}

template<typename T> bool imu_bno055::is_nan_or_inf(const std::vector<T> &arr) {
  for (size_t i = 0; i < arr.size(); i++) {
    if (std::isnan(arr[i]) || std::isinf(arr[i])) {
      return true;
    }
  }
  return false;
}

void imu_bno055::get_attitude_data(float data[3]) {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float raw_roll_deg  = -orientationData.orientation.z;
  float raw_pitch_deg = -orientationData.orientation.y;
  float raw_yaw_deg   = -orientationData.orientation.x;
  if (!initialized_) {
    initial_attitude_data_[0] = clamp_deg(raw_roll_deg);
    initial_attitude_data_[1] = clamp_deg(raw_pitch_deg);
    initial_attitude_data_[2] = clamp_deg(raw_pitch_deg);
    initialized_ = true;
  }

  constexpr float offset_roll_deg = 0.0;
  constexpr float offset_pitch_deg = 0.0;
  constexpr float offset_yaw_deg = 0.0;
  attitude_data_[0] = data[0] = clamp_deg(raw_roll_deg) - initial_attitude_data_[0] + offset_roll_deg;
  attitude_data_[1] = data[1] = clamp_deg(raw_pitch_deg) - initial_attitude_data_[1] + offset_pitch_deg ;
  attitude_data_[2] = data[2] = clamp_deg(raw_yaw_deg) - initial_attitude_data_[2] + offset_yaw_deg;

#ifdef DEBUG_IMU_ATTITUDE
  Serial.print("Euler orientation roll, pitch, yaw: ");
  Serial.print(data[0]);
  Serial.print(" ");
  Serial.print(data[1]);
  Serial.print(" ");
  Serial.println(data[2]);
#endif
}

void imu_bno055::get_accel_data(float data[3]) {
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data[0] = accelerometerData.acceleration.x;
  data[1] = accelerometerData.acceleration.y;
  data[2] = accelerometerData.acceleration.z;

#ifdef DEBUG_IMU_ACCELERATION
  Serial.print("Acceleration: ");
  Serial.print(data[0]);
  Serial.print(" ");
  Serial.print(data[1]);
  Serial.print(" ");
  Serial.println(data[2]);
#endif
}

void imu_bno055::get_angvel_data(float data[3]) {
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  angvel_data_[0] = data[0] = angVelocityData.gyro.x * RAD_TO_DEG;
  angvel_data_[1] = data[1] = angVelocityData.gyro.y * RAD_TO_DEG;
  angvel_data_[2] = data[2] = angVelocityData.gyro.z * RAD_TO_DEG;

#ifdef DEBUG_IMU_ANGVEL
  Serial.print("Gyro: ");
  Serial.print(data[0]);
  Serial.print(" ");
  Serial.print(data[1]);
  Serial.print(" ");
  Serial.println(data[2]);
#endif
}

void imu_bno055::calibrate() {
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}

void imu_bno055::emergency_stop(Arm &arm) {
  if (std::abs(attitude_data_[0]) > 70.0f || std::abs(attitude_data_[1]) > 70.0f) {
    arm.set_arm_status(false);
  }

  if (is_nan_or_inf(attitude_data_) || is_nan_or_inf(angvel_data_)) {
    arm.set_arm_status(false);
  }
}