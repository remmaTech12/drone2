#include "../include/imu_bno055.h"

imu_bno055::imu_bno055() {
}

void imu_bno055::setup() {
  while (!bno.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  const double SAMPLE_FREQUENCY = 1.0 / (SAMPLING_TIME_MS * 0.001);
  madgwick.begin(SAMPLE_FREQUENCY);
}

void imu_bno055::get_attitude_data(float data[3]) {
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  /*
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
  madgwick.updateIMU(angVelocityData.gyro.x, angVelocityData.gyro.y, angVelocityData.gyro.z,
                     accelerometerData.acceleration.x, accelerometerData.acceleration.y, accelerometerData.acceleration.z);
  data[0] = madgwick.getRoll();
  data[1] = madgwick.getPitch();
  data[2] = madgwick.getYaw();
  Serial.print("Roll: ");
  Serial.print(data[0]);
  Serial.print(" Pitch: ");
  Serial.print(data[1]);
  Serial.print(" Yaw: ");
  Serial.println(data[2]);
  */
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  data[2] = -orientationData.orientation.x;
  data[1] = -orientationData.orientation.y;
  data[0] = -orientationData.orientation.z;
  // clamp from -180 to 180
  // create a small macro
  // use while loop to clamp
  // please create a lambda function to clamp
  auto clamp = [](float value) -> double {
    while (value > 180.0) {
      value -= 360.0;
    }
    while (value < -180.0) {
      value += 360.0;
    }
    return value;
  };
  const double roll  = clamp(data[0]);
  const double pitch = clamp(data[1]);
  const double yaw   = clamp(data[2]);
  Serial.print("Euler: ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(yaw);
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