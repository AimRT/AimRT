// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <cstddef>
#include "util/exception.h"

namespace aimrt::util {

/**
 * @brief Dynamic latch that supports adding and removing counts at runtime
 *
 * DynamicLatch is a thread-safe synchronization primitive that allows:
 * - Dynamically increasing the count (Add)
 * - Decreasing the count (CountDown)
 * - Waiting until the count reaches zero (Wait)
 * - Rejecting new counts after being closed (Close)
 *
 * Thread Safety: All methods are thread-safe.
 */

class DynamicLatch {
 public:
  DynamicLatch() = default;
  ~DynamicLatch() = default;

  DynamicLatch(const DynamicLatch&) = delete;
  DynamicLatch& operator=(const DynamicLatch&) = delete;

  DynamicLatch(DynamicLatch&&) = delete;
  DynamicLatch& operator=(DynamicLatch&&) = delete;

  bool TryAdd(std::ptrdiff_t delta = 1) {
    std::ptrdiff_t current_state = state_.load(std::memory_order_acquire);
    while (true) {  // compare and swap
      if (IsClosed(current_state)) [[unlikely]] {
        return false;
      }

      std::ptrdiff_t new_state = current_state + delta;

      if (state_.compare_exchange_weak(current_state, new_state,
                                       std::memory_order_acq_rel,
                                       std::memory_order_acquire)) {
        return true;
      }
    }
  }

  void CountDown(std::ptrdiff_t delta = 1) {
    std::ptrdiff_t prev_state = state_.fetch_sub(delta, std::memory_order_acq_rel);

    std::ptrdiff_t prev_count = GetCount(prev_state);
    AIMRT_ASSERT(prev_count >= delta, "DynamicLatch::CountDown underflow.");

    if (prev_count == delta) {
      state_.notify_all();
    }
  }

  void Close() {
    std::ptrdiff_t current_state = state_.load(std::memory_order_acquire);
    while (true) {
      if (IsClosed(current_state)) [[unlikely]] {
        return;
      }

      std::ptrdiff_t new_state = current_state | CLOSED_FLAG;

      if (state_.compare_exchange_weak(current_state, new_state,
                                       std::memory_order_acq_rel,
                                       std::memory_order_acquire)) {
        if (GetCount(new_state) == 0) {
          state_.notify_all();
        }
        return;
      }
    }
  }

  void Wait() const {
    std::ptrdiff_t current_state = state_.load(std::memory_order_acquire);

    while (GetCount(current_state) != 0) {
      state_.wait(current_state, std::memory_order_acquire);
      current_state = state_.load(std::memory_order_acquire);
    }
  }

  void CloseAndWait() {
    Close();
    Wait();
  }

  std::ptrdiff_t GetCurrentCount() const {
    return GetCount(state_.load(std::memory_order_relaxed));
  }

  bool IsClosed() const {
    return IsClosed(state_.load(std::memory_order_acquire));
  }

 private:
  static std::ptrdiff_t GetCount(std::ptrdiff_t state) {
    return state & COUNT_MASK;
  }

  static bool IsClosed(std::ptrdiff_t state) {
    return (state & CLOSED_FLAG) != 0;
  }

 private:
  //  use the highest bit as the closed flag, and the low 63 bits are used for counting
  static constexpr std::ptrdiff_t CLOSED_FLAG = std::ptrdiff_t(1) << 63;
  static constexpr std::ptrdiff_t COUNT_MASK = ~CLOSED_FLAG;

  static_assert(sizeof(std::ptrdiff_t) == 8,
                "DynamicLatch requires 64-bit ptrdiff_t");

  static_assert(std::atomic<std::ptrdiff_t>::is_always_lock_free,
                "DynamicLatch requires lock-free atomic operations");

  mutable std::atomic<std::ptrdiff_t> state_{0};
};

}  // namespace aimrt::util