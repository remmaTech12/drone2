#include "../include/imu_bno055.h"

imu_bno055::imu_bno055() {
}

void imu_bno055::setup() {
  while (!bno.begin(OPERATION_MODE_IMUPLUS))
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  calibrate();
}

void imu_bno055::get_attitude_data(float data[3]) {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double raw_roll_deg  = -orientationData.orientation.z;
  double raw_pitch_deg = -orientationData.orientation.y;
  double raw_yaw_deg   = -orientationData.orientation.x;
  auto clamp = [](float value) -> double {
    while (value >  180.0) value -= 360.0;
    while (value < -180.0) value += 360.0;
    return value;
  };
  constexpr float offset_roll_deg = -5.0;
  constexpr float offset_pitch_deg = 0.1;
  data[0] = clamp(raw_roll_deg) - initial_attitude_data_[0] + offset_roll_deg;    // deg
  data[1] = clamp(raw_pitch_deg) - initial_attitude_data_[1] + offset_pitch_deg ; // deg
  data[2] = clamp(raw_yaw_deg) - initial_attitude_data_[2];   // deg

  if (!initialized_) {
    initial_attitude_data_[0] = clamp(raw_roll_deg);
    initial_attitude_data_[1] = clamp(raw_pitch_deg);
    initial_attitude_data_[2] = clamp(raw_pitch_deg);
    initialized_ = true;
  }

#ifdef DEBUG_IMU_ATTITUDE
  Serial.print("Euler: ");
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
}

void imu_bno055::get_angvel_data(float data[3]) {
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  data[0] = angVelocityData.gyro.x * RAD_TO_DEG;
  data[1] = angVelocityData.gyro.y * RAD_TO_DEG;
  data[2] = angVelocityData.gyro.z * RAD_TO_DEG;

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

void imu_bno055::printIMUEvent(sensors_event_t* event) {
  Serial.print("IMU data: ");
  double x = -1000000, y = -1000000,
         z = -1000000;  // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unknown data type:");
  }
  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}