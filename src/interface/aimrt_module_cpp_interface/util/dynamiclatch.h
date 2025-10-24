// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <atomic>
#include <barrier>
#include <latch>
#include <memory>
#include "util/exception.h"

#pragma once

class DynamicLatch {
    private:
     static constexpr std::ptrdiff_t CLOSED_FLAG = std::ptrdiff_t(1) << 63;
     static constexpr std::ptrdiff_t COUNT_MASK = ~CLOSED_FLAG;

     std::atomic<std::ptrdiff_t> state_{0};
     static std::ptrdiff_t get_count(std::ptrdiff_t state) {
       return state & COUNT_MASK;
     }

    public:
     DynamicLatch() = default;
     DynamicLatch(const DynamicLatch&) = delete;
     DynamicLatch& operator=(const DynamicLatch&) = delete;
     DynamicLatch(DynamicLatch&&) = delete;
     DynamicLatch& operator=(DynamicLatch&&) = delete;

     void Add(std::ptrdiff_t delta = 1) {
       std::ptrdiff_t current_state = state_.load(std::memory_order_relaxed);
       while (true) {
         if((current_state & CLOSED_FLAG) == 0) {
          return ;
         }

         std::ptrdiff_t new_state = current_state + delta;
         if (state_.compare_exchange_weak(current_state, new_state,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
           return;
         }
       }
     }

     void Done(std::ptrdiff_t delta = 1) {
       std::ptrdiff_t prev_state = state_.fetch_sub(delta, std::memory_order_acq_rel);
       std::ptrdiff_t prev_count = get_count(prev_state);

       AIMRT_ASSERT(prev_count >= delta, "DynamicLatch underflow in Done().");  // impossible to happen
       if (prev_count == delta) {
         state_.notify_all();
       }
     }

     void Close() {
       std::ptrdiff_t prev_state = state_.fetch_or(CLOSED_FLAG, std::memory_order_release);

       if (get_count(prev_state) == 0) {
         state_.notify_all();
       }
     }

     void Wait() const {
       std::ptrdiff_t current_state = state_.load(std::memory_order_acquire);

       while (get_count(current_state) != 0) {
         state_.wait(current_state, std::memory_order_acquire);
         current_state = state_.load(std::memory_order_acquire);
       }
     }
   };
