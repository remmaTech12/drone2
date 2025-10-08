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

void flow_pmw3901::calculate_velocity_position(double height) {
  double height_m = height / 1000.0;

  unsigned long current_ms = millis();
  const float dt = (current_ms - previous_ms) / 1000.0f;
  previous_ms = current_ms;

  constexpr float k = 0.0035;  // optical flow angular scale factor: original 0.021
  vx = dx * k * height_m / dt;
  vy = dy * k * height_m / dt;
  x += dx * k * height_m;
  y += dy * k * height_m;
}

void flow_pmw3901::printMotionCount() {
  //Serial.print("Optical Flow sensor output: dx: ");
  //Serial.print(" dx: ");
  Serial.println(dx);
  //Serial.print(" dy: ");
  Serial.println(dy);
}

void flow_pmw3901::get_position_data(float data[2]) {
  data[0] = x;
  data[1] = y;
}