// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/allocator/allocator_base.h"
#include "util/exception.h"

namespace aimrt::allocator {

class AllocatorRef {
 public:
  AllocatorRef() = default;
  explicit AllocatorRef(const aimrt_allocator_base_t* base_ptr)
      : base_ptr_(base_ptr) {}
  ~AllocatorRef() = default;

  explicit operator bool() const { return (base_ptr_ != nullptr); }

  const aimrt_allocator_base_t* NativeHandle() const { return base_ptr_; }

  void* GetThreadLocalBuf(size_t buf_size) {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    return base_ptr_->get_thread_local_buf(base_ptr_->impl, buf_size);
  }

 private:
  const aimrt_allocator_base_t* base_ptr_ = nullptr;
};

}  // namespace aimrt::allocator
