#pragma once
#include <deque>

class MovingAverage {
  public:
    MovingAverage() = default;
    void setup(int window_size) {
        window_size_ = window_size;
        data_.clear();
        sum_ = 0.0;
    }
    float filter(float data) {
        sum_ += data;
        data_.push_back(data);
        if (data_.size() > window_size_) {
            sum_ -= data_.front();
            data_.pop_front();
        }
        return sum_ / data_.size();
    }

  private:
    int window_size_ = 0;
    std::deque<float> data_;
    float sum_ = 0.0;
};