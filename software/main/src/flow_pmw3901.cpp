#include "../include/flow_pmw3901.h"

flow_pmw3901::flow_pmw3901() : sensor(PMW3901_CS_PIN) {
}

void flow_pmw3901::setup() {
  while (!sensor.begin()) {
    Serial.println("Ooops, no PMW3901 detected ... Check the SPI connection!");
  }
  Serial.println("PMW3901 sensor initialized successfully!");
}

void flow_pmw3901::readMotionCount(int data[2]) {
  sensor.readMotionCount(&dx, &dy);
  data[0] = dx;
  data[1] = dy;
}

void flow_pmw3901::printMotionCount() {
  Serial.print("Optical Flow sensor output: dx: ");
  Serial.print(dx);
  Serial.print(" dy: ");
  Serial.println(dy);
}