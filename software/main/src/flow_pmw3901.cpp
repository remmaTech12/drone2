#include "../include/flow_pmw3901.h"

flow_pmw3901::flow_pmw3901() : sensor(PMW3901_CS_PIN) {
}

void flow_pmw3901::setup() {
  while (!sensor.begin()) {
    Serial.println("Ooops, no PMW3901 detected ... Check the SPI connection!");
  }
  Serial.println("PMW3901 sensor initialized successfully!");

  constexpr int pixel_window_size = 1;
  constexpr int position_window_size = 1;
  pixel_dx_filter_.setup(pixel_window_size);
  pixel_dy_filter_.setup(pixel_window_size);
  position_dx_filter_.setup(position_window_size);
  position_dy_filter_.setup(position_window_size);
}

void flow_pmw3901::read_motion_count() {
  int16_t raw_pixel_dx, raw_pixel_dy;
  sensor.readMotionCount(&raw_pixel_dx, &raw_pixel_dy);

  // coordinate transformation
  pixel_dx_ = -raw_pixel_dy;
  pixel_dy_ = -raw_pixel_dx;

  pixel_dx_ = pixel_dx_filter_.filter(static_cast<float>(pixel_dx_));
  pixel_dy_ = pixel_dy_filter_.filter(static_cast<float>(pixel_dy_));

#ifdef DEBUG_FLOW_PIXEL_DELTA
  Serial.print("pixel_dx: ");
  Serial.println(pixel_dx_, 4);
  Serial.print("pixel_dy: ");
  Serial.println(pixel_dy_, 4);
#endif
}

void flow_pmw3901::calculate_delta_position(double distance, double height, float ang_data[3]) {
  const double roll_ang = ang_data[0] * DEG_TO_RAD;
  const double pitch_ang = ang_data[1] * DEG_TO_RAD;
  const double delta_roll_ang = roll_ang - pre_roll_ang_;
  const double delta_pitch_ang = pitch_ang - pre_pitch_ang_;
  pre_roll_ang_ = roll_ang;
  pre_pitch_ang_ = pitch_ang;

  // mm to m
  const double distance_m = distance / 1000.0;
  const double height_m = height / 1000.0;

  constexpr float k = 0.0035;  // optical flow angular scale factor: original 0.021
  position_dx_ = position_dx_filter_.filter(pixel_dx_ * k * height_m + delta_pitch_ang * distance_m);
  position_dy_ = position_dy_filter_.filter(pixel_dy_ * k * height_m - delta_roll_ang * distance_m);

#ifdef DEBUG_FLOW_POSITION_DELTA
//  Serial.print("position_dx: ");
  Serial.print(position_dx_, 4);
  Serial.print("\t");
//  Serial.print("position_dy: ");
  Serial.println(position_dy_, 4);
#endif
}

void flow_pmw3901::get_delta_position_data(float data[2]) {
  data[0] = position_dx_;
  data[1] = position_dy_;
}