// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdlib>
#include <cstring>

#include "aimrt_module_c_interface/util/buffer_base.h"

namespace aimrt::util {

class SimpleBufferArrayAllocator {
 public:
  static bool Reserve(aimrt_buffer_array_t* buffer_array, size_t new_cap) {
    aimrt_buffer_t* cur_data = buffer_array->data;

    buffer_array->data = new aimrt_buffer_t[new_cap];
    buffer_array->capacity = new_cap;

    if (cur_data) {
      memcpy(buffer_array->data, cur_data, buffer_array->len * sizeof(aimrt_buffer_t));
      delete[] cur_data;
    }

    return true;
  }

  static aimrt_buffer_t Allocate(aimrt_buffer_array_t* buffer_array, size_t size) {
    void* data = std::malloc(size);

    if (data == nullptr) [[unlikely]]
      return aimrt_buffer_t{nullptr, 0};

    // can directly put in current data
    if (buffer_array->capacity > buffer_array->len) {
      return (buffer_array->data[buffer_array->len++] = aimrt_buffer_t{data, size});
    }

    // current data area is full, need to reallocate space
    static constexpr size_t kInitCapacitySzie = 2;
    size_t new_capacity = (buffer_array->capacity < kInitCapacitySzie)
                              ? kInitCapacitySzie
                              : (buffer_array->capacity << 1);
    if (!Reserve(buffer_array, new_capacity)) [[unlikely]]
      return aimrt_buffer_t{nullptr, 0};

    return (buffer_array->data[buffer_array->len++] = aimrt_buffer_t{data, size});
  }

  static void Release(aimrt_buffer_array_t* buffer_array) {
    for (size_t ii = 0; ii < buffer_array->len; ++ii) {
      std::free(buffer_array->data[ii].data);
      buffer_array->data[ii].data = nullptr;
      buffer_array->data[ii].len = 0;
    }

    if (buffer_array->data) {
      delete[] buffer_array->data;
      buffer_array->data = nullptr;
      buffer_array->len = 0;
      buffer_array->capacity = 0;
    }
  }

  static const aimrt_buffer_array_allocator_t* NativeHandle() {
    static constexpr aimrt_buffer_array_allocator_t kSimpleBufferArrayAllocator{
        .reserve = [](void* impl, aimrt_buffer_array_t* buffer_array, size_t new_cap) -> bool {
          return Reserve(buffer_array, new_cap);  //
        },
        .allocate = [](void* impl, aimrt_buffer_array_t* buffer_array, size_t size) -> aimrt_buffer_t {
          return Allocate(buffer_array, size);
        },
        .release = [](void* impl, aimrt_buffer_array_t* buffer_array) {
          Release(buffer_array);  //
        },
        .impl = nullptr};

    return &kSimpleBufferArrayAllocator;
  }
};

class FlatBufferArrayAllocator {
 public:
  FlatBufferArrayAllocator(void* ptr, size_t size)
      : end_ptr_(static_cast<char*>(ptr) + size), cur_ptr_(ptr), base_(GenBase(this)) {}
  ~FlatBufferArrayAllocator() = default;

  FlatBufferArrayAllocator(const FlatBufferArrayAllocator&) = delete;
  FlatBufferArrayAllocator& operator=(const FlatBufferArrayAllocator&) = delete;

  const aimrt_buffer_array_allocator_t* NativeHandle() const { return &base_; }

  static bool Reserve(aimrt_buffer_array_t* buffer_array, size_t new_cap) {
    aimrt_buffer_t* cur_data = buffer_array->data;

    buffer_array->data = new aimrt_buffer_t[new_cap];
    buffer_array->capacity = new_cap;

    if (cur_data) {
      memcpy(buffer_array->data, cur_data, buffer_array->len * sizeof(aimrt_buffer_t));
      delete[] cur_data;
    }

    return true;
  }

  aimrt_buffer_t Allocate(aimrt_buffer_array_t* buffer_array, size_t size) {
    if (out_of_memory_) [[unlikely]]
      return aimrt_buffer_t{nullptr, 0};

    if (cur_ptr_ == nullptr) [[unlikely]] {
      out_of_memory_ = true;
      return aimrt_buffer_t{nullptr, 0};
    }

    void* last_ptr = cur_ptr_;
    cur_ptr_ = static_cast<char*>(cur_ptr_) + size;

    if (cur_ptr_ > end_ptr_) [[unlikely]] {
      out_of_memory_ = true;
      return aimrt_buffer_t{nullptr, 0};
    }

    // can directly put in current data
    if (buffer_array->capacity > buffer_array->len) {
      return (buffer_array->data[buffer_array->len++] = aimrt_buffer_t{last_ptr, size});
    }

    // current data area is full, need to reallocate space
    static constexpr size_t kInitCapacitySzie = 2;
    size_t new_capacity = (buffer_array->capacity < kInitCapacitySzie)
                              ? kInitCapacitySzie
                              : (buffer_array->capacity << 1);
    if (!Reserve(buffer_array, new_capacity)) [[unlikely]]
      return aimrt_buffer_t{nullptr, 0};

    return (buffer_array->data[buffer_array->len++] = aimrt_buffer_t{last_ptr, size});
  }

  static void Release(aimrt_buffer_array_t* buffer_array) {
    for (size_t ii = 0; ii < buffer_array->len; ++ii) {
      buffer_array->data[ii].data = nullptr;
      buffer_array->data[ii].len = 0;
    }

    if (buffer_array->data) {
      delete[] buffer_array->data;
      buffer_array->data = nullptr;
      buffer_array->len = 0;
      buffer_array->capacity = 0;
    }
  }

  bool IsOutOfMemory() const { return out_of_memory_; }

  static aimrt_buffer_array_allocator_t GenBase(void* impl) {
    return aimrt_buffer_array_allocator_t{
        .reserve = [](void* impl, aimrt_buffer_array_t* buffer_array, size_t new_cap) -> bool {
          return Reserve(buffer_array, new_cap);  //
        },
        .allocate = [](void* impl, aimrt_buffer_array_t* buffer_array, size_t size) -> aimrt_buffer_t {
          return static_cast<FlatBufferArrayAllocator*>(impl)->Allocate(buffer_array, size);
        },
        .release = [](void* impl, aimrt_buffer_array_t* buffer_array) {
          Release(buffer_array);  //
        },
        .impl = impl};
  }

 private:
  void* end_ptr_;
  void* cur_ptr_;

  bool out_of_memory_ = false;

  const aimrt_buffer_array_allocator_t base_;
};

}  // namespace aimrt::util
