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

void flow_pmw3901::calculate_velocity_position(double height, float ang_data[3]) {
  const unsigned long current_ms = millis();
  const float dt = (current_ms - previous_ms) / 1000.0f;
  previous_ms = current_ms;

  const double roll_ang = ang_data[0] * DEG_TO_RAD;
  const double pitch_ang = ang_data[1] * DEG_TO_RAD;
  const double delta_roll_ang = roll_ang - pre_roll_ang;
  const double delta_pitch_ang = pitch_ang - pre_pitch_ang;
  pre_roll_ang = roll_ang;
  pre_pitch_ang = pitch_ang;
  const double distance_m = height / 1000.0;
  const double height_m = height / 1000.0 * cos(roll_ang) * cos(pitch_ang);

  constexpr float k = 0.0035;  // optical flow angular scale factor: original 0.021
  vx = dx * k * height_m / dt;
  vy = dy * k * height_m / dt;
  x += dx * k * height_m + delta_pitch_ang * distance_m;
  y += dy * k * height_m - delta_roll_ang * distance_m;

  /*
  Serial.print("x: ");
  Serial.println(x);
  Serial.print("y: ");
  Serial.println(y);
  */
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