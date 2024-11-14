// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "aimrt_module_c_interface/util/buffer_base.h"

namespace aimrt::util {

class ZenohBufferArrayAllocator {
 public:
  ZenohBufferArrayAllocator(uint64_t z_shm_capacity, void* z_shm_head_ptr)
      : z_shm_capacity_(z_shm_capacity),
        z_shm_head_ptr_(z_shm_head_ptr),
        z_shm_cur_ptr_(z_shm_head_ptr),
        base_(GenBase(this)) {}

  ~ZenohBufferArrayAllocator() = default;
  ZenohBufferArrayAllocator(const ZenohBufferArrayAllocator&) = delete;
  ZenohBufferArrayAllocator& operator=(const ZenohBufferArrayAllocator&) = delete;
  const aimrt_buffer_array_allocator_t* NativeHandle() const { return &base_; }

  void Reserve(aimrt_buffer_array_t* buffer_array, size_t new_cap) {
    aimrt_buffer_t* cur_data = buffer_array->data;

    buffer_array->data = new aimrt_buffer_t[new_cap];
    buffer_array->capacity = new_cap;

    if (cur_data) {
      memcpy(buffer_array->data, cur_data, buffer_array->len * sizeof(aimrt_buffer_t));
      delete[] cur_data;
    }
  }

  aimrt_buffer_t Allocate(aimrt_buffer_array_t* buffer_array, size_t size) {
    void* data = z_shm_cur_ptr_;
    z_shm_cur_ptr_ = static_cast<char*>(z_shm_cur_ptr_) + size;
    z_shm_used_size_ += size;

    if (z_shm_used_size_ > z_shm_capacity_) [[unlikely]] {
      z_is_shm_enough_ = false;
      return aimrt_buffer_t{nullptr, 0};
    }

    if (data == nullptr) [[unlikely]]
      return aimrt_buffer_t{nullptr, 0};

    if (buffer_array->capacity > buffer_array->len) {
      return (buffer_array->data[buffer_array->len++] = aimrt_buffer_t{data, size});
    }

    static constexpr size_t kInitCapacitySzie = 2;
    size_t new_capacity = (buffer_array->capacity < kInitCapacitySzie)
                              ? kInitCapacitySzie
                              : (buffer_array->capacity << 1);

    Reserve(buffer_array, new_capacity);

    return (buffer_array->data[buffer_array->len++] = aimrt_buffer_t{data, size});
  }

  void Release(aimrt_buffer_array_t* buffer_array) {
    if (buffer_array->data) delete[] buffer_array->data;
  }

  bool IsShmEnough() { return z_is_shm_enough_; }

  static const aimrt_buffer_array_allocator_t GenBase(void* impl) {
    return aimrt_buffer_array_allocator_t{
        .reserve = [](void* impl, aimrt_buffer_array_t* buffer_array, size_t new_cap) {
          static_cast<ZenohBufferArrayAllocator*>(impl)->Reserve(buffer_array, new_cap);  //
        },
        .allocate = [](void* impl, aimrt_buffer_array_t* buffer_array, size_t size) -> aimrt_buffer_t {
          return static_cast<ZenohBufferArrayAllocator*>(impl)->Allocate(buffer_array, size);
        },
        .release = [](void* impl, aimrt_buffer_array_t* buffer_array) {
          static_cast<ZenohBufferArrayAllocator*>(impl)->Release(buffer_array);  //
        },
        .impl = impl};
  }

 private:
  const aimrt_buffer_array_allocator_t base_;

  void* z_shm_head_ptr_;
  void* z_shm_cur_ptr_;

  uint64_t z_shm_capacity_;
  uint64_t z_shm_used_size_ = 0;

  bool z_is_shm_enough_ = true;
};

}  // namespace aimrt::util