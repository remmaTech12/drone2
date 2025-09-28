#include "../include/tof_vl53l1x.h"

tof_vl53l1x::tof_vl53l1x() {
}

void tof_vl53l1x::setup() {
  sensor.setTimeout(500);
  while (!sensor.init()) {
    Serial.println("Ooops, no VL53L1X detected ... Check your wiring or I2C ADDR!");
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
}

void tof_vl53l1x::readDistance(int16_t &dist) {
  dist = sensor.read();
  distance = dist;
}

void tof_vl53l1x::printDistance() {
  Serial.print("VL53L1X sensor output: distance: ");
  Serial.print(distance);
  Serial.println(" mm");
}