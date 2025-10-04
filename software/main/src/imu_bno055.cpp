#include "../include/imu_bno055.h"

imu_bno055::imu_bno055() {
}

void imu_bno055::setup() {
  while (!bno.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
}

static inline void quatToRPY_ZYX(const imu::Quaternion& q_in,
  float& roll_deg, float& pitch_deg, float& yaw_deg)
{
  float w = q_in.w(), x = q_in.x(), y = q_in.y(), z = q_in.z();
  float n = sqrtf(w*w + x*x + y*y + z*z);
  if (n > 0.f) { w/=n; x/=n; y/=n; z/=n; }
  
  float sinp = 2.f*(w*y - z*x);
  if (sinp >  1.f) sinp =  1.f;
  if (sinp < -1.f) sinp = -1.f;
  
  float roll  = atan2f(2.f*(w*x + y*z), 1.f - 2.f*(x*x + y*y));
  float pitch = asinf(sinp);
  float yaw   = atan2f(2.f*(w*z + x*y), 1.f - 2.f*(y*y + z*z));
  
  const float R2D = 180.0f / 3.14159265358979323846f;
  roll_deg  = roll  * R2D;
  pitch_deg = pitch * R2D;
  yaw_deg   = yaw   * R2D;
}


void imu_bno055::get_attitude_data(float data[3]) {
  /*
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print("angvel: ");
  Serial.print(angVelocityData.gyro.x);
  Serial.print(" ");
  Serial.print(angVelocityData.gyro.y);
  Serial.print(" ");
  Serial.println(angVelocityData.gyro.z);
  Serial.print("accel: ");
  Serial.print(accelerometerData.acceleration.x);
  Serial.print(" ");
  Serial.print(accelerometerData.acceleration.y);
  Serial.print(" ");
  Serial.println(accelerometerData.acceleration.z);
  */
  imu::Quaternion q = bno.getQuat();
  float roll_deg, pitch_deg, yaw_deg;
  quatToRPY_ZYX(q, roll_deg, pitch_deg, yaw_deg);
  //sensors_event_t orientationData;
  //bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  //double roll_deg  = -orientationData.orientation.z;
  //double pitch_deg = -orientationData.orientation.y;
  //double yaw_deg   = -orientationData.orientation.x;
  auto clamp = [](float value) -> double {
    while (value >  180.0) value -= 360.0;
    while (value < -180.0) value += 360.0;
    return value;
  };
  roll_deg  = clamp(roll_deg);
  pitch_deg = clamp(pitch_deg);
  yaw_deg   = clamp(yaw_deg);
  Serial.print("Euler: ");
  Serial.print(roll_deg);
  Serial.print(" ");
  Serial.print(pitch_deg);
  Serial.print(" ");
  Serial.println(yaw_deg);

  data[0] = roll_deg * DEG_TO_RAD;
  data[1] = pitch_deg * DEG_TO_RAD;
  data[2] = yaw_deg * DEG_TO_RAD;
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
  data[0] = angVelocityData.gyro.x;
  data[1] = angVelocityData.gyro.y;
  data[2] = angVelocityData.gyro.z;
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