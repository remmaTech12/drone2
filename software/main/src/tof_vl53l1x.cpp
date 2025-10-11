#include "../include/tof_vl53l1x.h"

tof_vl53l1x::tof_vl53l1x() {
}

void tof_vl53l1x::setup() {
  Wire.setClock(400000);
  sensor_.setTimeout(500);
  while (!sensor_.init()) {
    Serial.println("Ooops, no VL53L1X detected ... Check your wiring or I2C ADDR!");
  }
  sensor_.setDistanceMode(VL53L1X::Long);
  sensor_.setMeasurementTimingBudget(40000);
  sensor_.startContinuous(60);
}

void tof_vl53l1x::read_distance() {
  if (sensor_.dataReady()) {
    distance_ = sensor_.read();
  }
}

float tof_vl53l1x::get_distance() {
#ifdef DEBUG_TOF_DISTANCE
  Serial.print("VL53L1X sensor output: distance: ");
  Serial.print(distance_);
  Serial.println(" mm");
#endif

  return distance_;
}

float tof_vl53l1x::get_height(float ang_data[3]) {
  const float roll_ang_rad  = ang_data[0] * DEG_TO_RAD;
  const float pitch_ang_rad = ang_data[1] * DEG_TO_RAD;
  const float height = distance_ * cos(roll_ang_rad) * cos(pitch_ang_rad);
#ifdef DEBUG_TOF_HEIGHT
  Serial.print("VL53L1X sensor output: height: ");
  Serial.print(height);
  Serial.println(" mm");
#endif

  return height;
}