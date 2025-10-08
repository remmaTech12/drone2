#pragma once

class LowPassFilter {
  public:
    LowPassFilter() = default;

    void setup(float kpre) {
      // float Tsamp = SAMPLING_TIME_MS / 1000.0f;
      // float tau   = 1.0f / (2.0f * M_PI * cutoff_freq);
      // float kpre  = tau / (Tsamp + tau);
      // reference: https://qiita.com/motorcontrolman/items/39d4abc6c4862817e646
      // cutoff_freq = 1000: kpre = 0.0157
      kpre_ = kpre;
    }
    float filter(float cur_data)
    {
      float filtered_data = kpre_*pre_filtered_data_ + (1.0f - kpre_)*cur_data;
      pre_filtered_data_ = filtered_data;

      return filtered_data;
    }

  private:
    float pre_filtered_data_ = 0.0;
    float kpre_ = 0.0;
};
