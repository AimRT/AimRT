// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>

#include "aimrt_module_c_interface/util/buffer_base.h"

#include <google/protobuf/io/zero_copy_stream.h>

namespace aimrt {

class BufferArrayZeroCopyOutputStream
    : public ::google::protobuf::io::ZeroCopyOutputStream {
 public:
  BufferArrayZeroCopyOutputStream(
      aimrt_buffer_array_t* buffer_array_ptr,
      const aimrt_buffer_array_allocator_t* allocator_ptr)
      : buffer_array_ptr_(buffer_array_ptr),
        allocator_ptr_(allocator_ptr) {}

  virtual ~BufferArrayZeroCopyOutputStream() = default;

  bool Next(void** data, int* size) override {
    if (cur_buf_used_size_ == cur_block_size_) {
      aimrt_buffer_t new_buffer = allocator_ptr_->allocate(
          allocator_ptr_->impl, buffer_array_ptr_, cur_block_size_ <<= 1);
      if (new_buffer.data == nullptr) [[unlikely]]
        return false;
      *data = new_buffer.data;
      byte_count_ += (*size = static_cast<int>(cur_buf_used_size_ = cur_block_size_));
    } else {
      *data =
          static_cast<char*>(buffer_array_ptr_->data[buffer_array_ptr_->len - 1].data) +
          cur_buf_used_size_;
      byte_count_ += (*size = static_cast<int>(cur_block_size_ - cur_buf_used_size_));
      cur_buf_used_size_ = cur_block_size_;
    }
    return true;
  }

  void BackUp(int count) override {
    cur_buf_used_size_ -= count;
    byte_count_ -= count;
  }

  int64_t ByteCount() const override { return byte_count_; }

  void CommitLastBuf() {
    if (buffer_array_ptr_->len > 0) {
      buffer_array_ptr_->data[buffer_array_ptr_->len - 1].len = cur_buf_used_size_;
    }
  }

 private:
  static constexpr size_t kInitBlockSize = 1024;

  aimrt_buffer_array_t* buffer_array_ptr_;
  const aimrt_buffer_array_allocator_t* allocator_ptr_;
  size_t cur_block_size_ = kInitBlockSize / 2;
  size_t cur_buf_used_size_ = kInitBlockSize / 2;
  int64_t byte_count_ = 0;
};

class BufferArrayZeroCopyInputStream
    : public ::google::protobuf::io::ZeroCopyInputStream {
 public:
  explicit BufferArrayZeroCopyInputStream(
      aimrt_buffer_array_view_t buffer_array_view)
      : buffer_array_view_(buffer_array_view) {}

  virtual ~BufferArrayZeroCopyInputStream() = default;

  bool Next(const void** data, int* size) override {
    if (cur_buf_index_ >= buffer_array_view_.len) return false;

    if (cur_buf_unused_size_ == 0) {
      const auto& cur_buffer = buffer_array_view_.data[cur_buf_index_++];
      *data = cur_buffer.data;
      byte_count_ += (*size = cur_buffer.len);
    } else {
      const auto& cur_buffer = buffer_array_view_.data[cur_buf_index_ - 1];
      *data = static_cast<const char*>(cur_buffer.data) + (cur_buffer.len - cur_buf_unused_size_);
      byte_count_ += (*size = cur_buf_unused_size_);
      cur_buf_unused_size_ = 0;
    }

    return true;
  }

  void BackUp(int count) override {
    cur_buf_unused_size_ += count;
    byte_count_ -= count;
  }

  bool Skip(int count) override {
    byte_count_ += count;

    const size_t buffer_vec_size = buffer_array_view_.len;

    for (;;) {
      if (static_cast<int>(cur_buf_unused_size_) > count) {
        cur_buf_unused_size_ -= count;
        count = 0;
        break;
      }

      count -= cur_buf_unused_size_;

      if (cur_buf_index_ >= buffer_vec_size) break;

      cur_buf_unused_size_ = buffer_array_view_.data[cur_buf_index_++].len;
    }

    return (count == 0);
  }

  int64_t ByteCount() const override { return byte_count_; }

 private:
  aimrt_buffer_array_view_t buffer_array_view_;
  size_t cur_buf_unused_size_ = 0;
  size_t cur_buf_index_ = 0;
  int64_t byte_count_ = 0;
};

}  // namespace aimrt
