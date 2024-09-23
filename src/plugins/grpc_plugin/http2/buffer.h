// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>

namespace aimrt::plugins::grpc_plugin::http2 {

class SimpleBuffer {
 public:
  explicit SimpleBuffer(size_t capacity) {
    data_.reserve(capacity);
  }
  ~SimpleBuffer() = default;

  void Write(const uint8_t* data, size_t len) {
    data_.append(reinterpret_cast<const char*>(data), len);
  }

  void Consume(size_t len) {
    read_pos_ += len;
  }

  // Read data to buffer.
  // Attention: The caller should ensure len <= GetReadableSize().
  void Read(uint8_t* data, size_t len) {
    assert(len <= GetReadableSize());
    std::memcpy(data, data_.data() + read_pos_, len);
  }

  [[nodiscard]] std::string_view GetStringView() const {
    return {data_.data() + read_pos_, data_.size() - read_pos_};
  }

  [[nodiscard]] size_t GetReadableSize() const { return data_.size() - read_pos_; }

  [[nodiscard]] bool Empty() const { return GetReadableSize() == 0; }

 private:
  std::string data_;
  size_t read_pos_ = 0;
};

}  // namespace aimrt::plugins::grpc_plugin::http2
