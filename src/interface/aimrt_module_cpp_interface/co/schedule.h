// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_EXECUTOR_USE_STDEXEC

  #include <exec/timed_scheduler.hpp>
  #include <stdexec/execution.hpp>

namespace aimrt::co {

inline constexpr auto& Schedule = stdexec::schedule;
inline constexpr auto& ScheduleAfter = exec::schedule_after;
inline constexpr auto& ScheduleAt = exec::schedule_at;

}  // namespace aimrt::co

#else

  #include <unifex/scheduler_concepts.hpp>

namespace aimrt::co {

inline constexpr auto& Schedule = unifex::schedule;
inline constexpr auto& ScheduleAfter = unifex::schedule_after;
inline constexpr auto& ScheduleAt = unifex::schedule_at;

}  // namespace aimrt::co

#endif