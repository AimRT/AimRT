// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <vector>

#include "aimrt_module_c_interface/util/buffer_base.h"
#include "aimrt_module_cpp_interface/util/buffer_array_allocator.h"

namespace aimrt::util {

class BufferArrayAllocatorRef {
 public:
  BufferArrayAllocatorRef() = default;
  explicit BufferArrayAllocatorRef(const aimrt_buffer_array_allocator_t* base_ptr)
      : base_ptr_(base_ptr) {}
  ~BufferArrayAllocatorRef() = default;

  explicit operator bool() const { return (base_ptr_ != nullptr); }

  const aimrt_buffer_array_allocator_t* NativeHandle() const {
    return base_ptr_;
  }

  void Reserve(aimrt_buffer_array_t* buffer_array, size_t new_cap) {
    base_ptr_->reserve(base_ptr_->impl, buffer_array, new_cap);
  }

  aimrt_buffer_t Allocate(aimrt_buffer_array_t* buffer_array, size_t size) {
    return base_ptr_->allocate(base_ptr_->impl, buffer_array, size);
  }

  void Release(aimrt_buffer_array_t* buffer_array) {
    base_ptr_->release(base_ptr_->impl, buffer_array);
  }

 private:
  const aimrt_buffer_array_allocator_t* base_ptr_ = nullptr;
};

class BufferArray {
 public:
  explicit BufferArray(
      const aimrt_buffer_array_allocator_t* allocator = SimpleBufferArrayAllocator::NativeHandle())
      : buffer_array_(aimrt_buffer_array_t{.data = nullptr, .len = 0, .capacity = 0}),
        allocator_ptr_(allocator) {}

  explicit BufferArray(BufferArrayAllocatorRef allocator)
      : buffer_array_(aimrt_buffer_array_t{.data = nullptr, .len = 0, .capacity = 0}),
        allocator_ptr_(allocator.NativeHandle()) {}

  ~BufferArray() { allocator_ptr_->release(allocator_ptr_->impl, &buffer_array_); }

  BufferArray(const BufferArray&) = delete;
  BufferArray& operator=(const BufferArray&) = delete;

  const aimrt_buffer_array_t* BufferArrayNativeHandle() const { return &buffer_array_; }
  aimrt_buffer_array_t* BufferArrayNativeHandle() { return &buffer_array_; }

  const aimrt_buffer_array_allocator_t* AllocatorNativeHandle() const { return allocator_ptr_; }

  size_t Size() const { return buffer_array_.len; }

  size_t Capacity() const { return buffer_array_.capacity; }

  aimrt_buffer_t* Data() const { return buffer_array_.data; }

  void Reserve(size_t new_cap) {
    allocator_ptr_->reserve(allocator_ptr_->impl, &buffer_array_, new_cap);
  }

  aimrt_buffer_t NewBuffer(size_t size) {
    return allocator_ptr_->allocate(allocator_ptr_->impl, &buffer_array_, size);
  }

  size_t BufferSize() const {
    size_t result = 0;
    for (size_t ii = 0; ii < buffer_array_.len; ++ii) {
      result += buffer_array_.data[ii].len;
    }
    return result;
  }

  std::string JoinToString() const {
    std::string result;
    for (size_t ii = 0; ii < buffer_array_.len; ++ii) {
      result += std::string_view(
          static_cast<const char*>(buffer_array_.data[ii].data),
          buffer_array_.data[ii].len);
    }
    return result;
  }

 private:
  aimrt_buffer_array_t buffer_array_;
  const aimrt_buffer_array_allocator_t* allocator_ptr_;
};

class BufferArrayView {
 public:
  explicit BufferArrayView(const std::vector<aimrt_buffer_view_t>& vec) {
    buffer_array_view_vec_ = vec;

    SyncCType();
  }

  explicit BufferArrayView(const void* data, size_t len) {
    buffer_array_view_vec_.emplace_back(
        aimrt_buffer_view_t{.data = data, .len = len});

    SyncCType();
  }

  explicit BufferArrayView(aimrt_buffer_t buffer) {
    buffer_array_view_vec_.emplace_back(
        aimrt_buffer_view_t{.data = buffer.data, .len = buffer.len});

    SyncCType();
  }

  explicit BufferArrayView(aimrt_buffer_view_t buffer_view) {
    buffer_array_view_vec_.emplace_back(buffer_view);

    SyncCType();
  }

  explicit BufferArrayView(aimrt_buffer_array_view_t buffer_array_view) {
    buffer_array_view_vec_.reserve(buffer_array_view.len);
    for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
      buffer_array_view_vec_.emplace_back(buffer_array_view.data[ii]);
    }

    SyncCType();
  }

  explicit BufferArrayView(aimrt_buffer_array_t buffer_array) {
    buffer_array_view_vec_.reserve(buffer_array.len);
    for (size_t ii = 0; ii < buffer_array.len; ++ii) {
      aimrt_buffer_t buffer = buffer_array.data[ii];
      buffer_array_view_vec_.emplace_back(
          aimrt_buffer_view_t{.data = buffer.data, .len = buffer.len});
    }

    SyncCType();
  }

  explicit BufferArrayView(const BufferArray& buffer_array) {
    size_t len = buffer_array.Size();
    aimrt_buffer_t* data = buffer_array.Data();

    buffer_array_view_vec_.reserve(len);
    for (size_t ii = 0; ii < len; ++ii) {
      aimrt_buffer_t buffer = data[ii];
      buffer_array_view_vec_.emplace_back(
          aimrt_buffer_view_t{.data = buffer.data, .len = buffer.len});
    }

    SyncCType();
  }

  ~BufferArrayView() = default;

  BufferArrayView(const BufferArrayView&) = delete;
  BufferArrayView& operator=(const BufferArrayView&) = delete;

  const aimrt_buffer_array_view_t* NativeHandle() const { return &buffer_array_view_; }
  aimrt_buffer_array_view_t* NativeHandle() { return &buffer_array_view_; }

  size_t Size() const { return buffer_array_view_.len; }

  const aimrt_buffer_view_t* Data() const { return buffer_array_view_.data; }

  size_t BufferSize() const {
    size_t result = 0;
    for (size_t ii = 0; ii < buffer_array_view_.len; ++ii) {
      result += buffer_array_view_.data[ii].len;
    }
    return result;
  }

  std::string JoinToString() const {
    std::string result;
    result.reserve(BufferSize());

    for (size_t ii = 0; ii < buffer_array_view_.len; ++ii) {
      result += std::string_view(
          static_cast<const char*>(buffer_array_view_.data[ii].data),
          buffer_array_view_.data[ii].len);
    }
    return result;
  }

  std::vector<char> JoinToCharVector() const {
    std::vector<char> result;
    result.resize(BufferSize());

    char* p = result.data();
    for (size_t ii = 0; ii < buffer_array_view_.len; ++ii) {
      memcpy(p, buffer_array_view_.data[ii].data, buffer_array_view_.data[ii].len);
      p += buffer_array_view_.data[ii].len;
    }
    return result;
  }

 private:
  void SyncCType() {
    buffer_array_view_.data = buffer_array_view_vec_.data();
    buffer_array_view_.len = buffer_array_view_vec_.size();
  }

 private:
  std::vector<aimrt_buffer_view_t> buffer_array_view_vec_;

  aimrt_buffer_array_view_t buffer_array_view_;
};

}  // namespace aimrt::util
