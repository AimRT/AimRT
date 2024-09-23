// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdlib>

#include "aimrt_module_c_interface/allocator/allocator_base.h"

namespace aimrt::runtime::core::allocator {

class AllocatorProxy {
 public:
  explicit AllocatorProxy()
      : base_(GenBase(this)) {}
  ~AllocatorProxy() = default;

  AllocatorProxy(const AllocatorProxy&) = delete;
  AllocatorProxy& operator=(const AllocatorProxy&) = delete;

  const aimrt_allocator_base_t* NativeHandle() const { return &base_; }

 private:
  static void* GetThreadLocalBuf(size_t buf_size) {
    constexpr size_t kMaxThreadLocalBufSize = 1024 * 1024 * 16;
    constexpr size_t kMinThreadLocalBufSize = 1024 * 4;

    if (buf_size > kMaxThreadLocalBufSize) return nullptr;

    struct Buf {
      Buf() : buf(malloc(kMinThreadLocalBufSize)), size(kMinThreadLocalBufSize) {}
      ~Buf() {
        if (buf != nullptr) free(buf);
      }
      void* buf;
      size_t size;
    };

    thread_local Buf thread_local_buf;

    if (thread_local_buf.size >= buf_size)
      return thread_local_buf.buf;

    if (thread_local_buf.buf != nullptr)
      free(thread_local_buf.buf);

    while (thread_local_buf.size < buf_size)
      thread_local_buf.size <<= 1;

    thread_local_buf.buf = malloc(thread_local_buf.size);

    return thread_local_buf.buf;
  }

  static aimrt_allocator_base_t GenBase(void* impl) {
    return aimrt_allocator_base_t{
        .get_thread_local_buf = [](void* impl, size_t buf_size) -> void* {
          return AllocatorProxy::GetThreadLocalBuf(buf_size);
        },
        .impl = impl};
  }

 private:
  const aimrt_allocator_base_t base_;
};

}  // namespace aimrt::runtime::core::allocator
