#include "../include/flow_pmw3901.h"

flow_pmw3901::flow_pmw3901() : sensor(PMW3901_CS_PIN) {
}

void flow_pmw3901::setup() {
  while (!sensor.begin()) {
    Serial.println("Ooops, no PMW3901 detected ... Check the SPI connection!");
  }
  Serial.println("PMW3901 sensor initialized successfully!");

  constexpr int window_size = 10;
  dx_filter.setup(window_size);
  dy_filter.setup(window_size);
}

void flow_pmw3901::readMotionCount(int data[2]) {
  int16_t tmp_dx, tmp_dy;
  sensor.readMotionCount(&tmp_dx, &tmp_dy);

  // coordinate transformation
  dx = -tmp_dy;
  dy = -tmp_dx;

  dx = dx_filter.filter(static_cast<float>(dx));
  dy = dy_filter.filter(static_cast<float>(dy));

  // store the data
  data[0] = dx;
  data[1] = dy;
}

void flow_pmw3901::printMotionCount() {
  //Serial.print("Optical Flow sensor output: dx: ");
  //Serial.print(" dx: ");
  //Serial.println(dx);
  //Serial.print(" dy: ");
  Serial.println(dy);
}