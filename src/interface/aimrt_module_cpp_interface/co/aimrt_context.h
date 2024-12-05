// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_EXECUTOR_USE_STDEXEC

  #include <exec/timed_scheduler.hpp>
  #include <stdexec/execution.hpp>

  #include "aimrt_module_cpp_interface/executor/executor_manager.h"
  #include "aimrt_module_cpp_interface/util/function.h"

namespace aimrt::co {

// Scheduler
class AimRTScheduler {
 public:
  // OperationState
  template <typename Receiver>
    requires stdexec::receiver<Receiver>
  struct OperationState final {
    template <typename Receiver2>
      requires std::constructible_from<Receiver, Receiver2>
    OperationState(executor::ExecutorRef executor_ref, Receiver2&& r)  //
        noexcept(std::is_nothrow_constructible_v<Receiver, Receiver2>)
        : executor_ref_(executor_ref), receiver_((Receiver2 &&) r) {}

    friend void tag_invoke(stdexec::start_t, OperationState& op) noexcept {
      op.executor_ref_.Execute([r{(Receiver &&) op.receiver_}]() mutable {
        try {
          stdexec::set_value((Receiver &&) r);
        } catch (...) {
          stdexec::set_error((Receiver &&) r, std::current_exception());
        }
      });
    }

    executor::ExecutorRef executor_ref_;
    Receiver receiver_;
  };

  // Sender
  class Task {
   public:
    using is_sender = void;
    using completion_signatures = stdexec::completion_signatures<
        stdexec::set_value_t(),
        stdexec::set_error_t(std::exception_ptr)>;

    explicit Task(executor::ExecutorRef executor_ref) noexcept
        : executor_ref_(executor_ref) {}

    template <class R>
    friend auto tag_invoke(stdexec::connect_t, const Task& self, R&& rec)  //
        noexcept(stdexec::__nothrow_constructible_from<stdexec::__decay_t<R>, R>) {
      return OperationState<std::remove_cvref_t<R>>(self.executor_ref_, (R &&) rec);
    }

    struct Env {
      executor::ExecutorRef executor_ref_;

      template <class CPO>
      friend AimRTScheduler
      tag_invoke(stdexec::get_completion_scheduler_t<CPO>, const Env& self) noexcept {
        return AimRTScheduler(self.executor_ref_);
      }
    };

    friend Env tag_invoke(stdexec::get_env_t, const Task& self) noexcept {
      return Env{self.executor_ref_};
    }

   private:
    executor::ExecutorRef executor_ref_;
  };

  // OperationState
  template <typename Receiver>
    requires stdexec::receiver<Receiver>
  struct SchedulerAtOperationState final {
    template <typename Receiver2>
      requires std::constructible_from<Receiver, Receiver2>
    SchedulerAtOperationState(
        executor::ExecutorRef executor_ref,
        std::chrono::system_clock::time_point tp,
        Receiver2&& r)  //
        noexcept(std::is_nothrow_constructible_v<Receiver, Receiver2>)
        : executor_ref_(executor_ref), tp_(tp), receiver_((Receiver2 &&) r) {}

    friend void tag_invoke(stdexec::start_t, SchedulerAtOperationState& op) noexcept {
      op.executor_ref_.ExecuteAt(op.tp_, [r{(Receiver &&) op.receiver_}]() mutable {
        try {
          stdexec::set_value((Receiver &&) r);
        } catch (...) {
          stdexec::set_error((Receiver &&) r, std::current_exception());
        }
      });
    }

   private:
    executor::ExecutorRef executor_ref_;
    std::chrono::system_clock::time_point tp_;
    Receiver receiver_;
  };

  // Sender
  class SchedulerAtTask {
   public:
    using is_sender = void;
    using completion_signatures = stdexec::completion_signatures<
        stdexec::set_value_t(),
        stdexec::set_error_t(std::exception_ptr)>;

    SchedulerAtTask(
        executor::ExecutorRef executor_ref,
        std::chrono::system_clock::time_point tp) noexcept
        : executor_ref_(executor_ref), tp_(tp) {}

    template <class R>
    friend auto tag_invoke(stdexec::connect_t, const SchedulerAtTask& self, R&& rec)  //
        noexcept(stdexec::__nothrow_constructible_from<stdexec::__decay_t<R>, R>) {
      return SchedulerAtOperationState<std::remove_cvref_t<R>>(self.executor_ref_, self.tp_, (R &&) rec);
    }

    struct Env {
      executor::ExecutorRef executor_ref_;

      template <class CPO>
      friend AimRTScheduler
      tag_invoke(stdexec::get_completion_scheduler_t<CPO>, const Env& self) noexcept {
        return AimRTScheduler(self.executor_ref_);
      }
    };

    friend Env tag_invoke(stdexec::get_env_t, const SchedulerAtTask& self) noexcept {
      return Env{self.executor_ref_};
    }

   private:
    executor::ExecutorRef executor_ref_;
    std::chrono::system_clock::time_point tp_;
  };

 public:
  explicit AimRTScheduler(executor::ExecutorRef executor_ref) noexcept
      : executor_ref_(executor_ref) {}

  friend Task
  tag_invoke(stdexec::schedule_t, const AimRTScheduler& s) noexcept {
    return Task(s.executor_ref_);
  }

  friend std::chrono::system_clock::time_point
  tag_invoke(exec::now_t, const AimRTScheduler& s) noexcept {
    return s.executor_ref_.Now();
  }

  friend SchedulerAtTask
  tag_invoke(exec::schedule_after_t,
             const AimRTScheduler& s,
             std::chrono::nanoseconds dt) noexcept {
    return SchedulerAtTask(s.executor_ref_, dt + s.executor_ref_.Now());
  }

  friend SchedulerAtTask
  tag_invoke(exec::schedule_at_t,
             const AimRTScheduler& s,
             std::chrono::system_clock::time_point tp) noexcept {
    return SchedulerAtTask(s.executor_ref_, tp);
  }

  friend bool operator==(const AimRTScheduler& a, const AimRTScheduler& b) noexcept {
    return a.executor_ref_.NativeHandle() == b.executor_ref_.NativeHandle();
  }

  friend bool operator!=(const AimRTScheduler& a, const AimRTScheduler& b) noexcept {
    return a.executor_ref_.NativeHandle() != b.executor_ref_.NativeHandle();
  }

  explicit operator bool() const { return static_cast<bool>(executor_ref_); }

 private:
  executor::ExecutorRef executor_ref_;
};

// Context
class AimRTContext {
 public:
  explicit AimRTContext(executor::ExecutorManagerRef executor_manager_ref = {}) noexcept
      : executor_manager_ref_(executor_manager_ref) {}

  AimRTScheduler GetScheduler(std::string_view executor_name) {
    return AimRTScheduler(executor_manager_ref_.GetExecutor(executor_name));
  }

  explicit operator bool() const { return static_cast<bool>(executor_manager_ref_); }

 private:
  executor::ExecutorManagerRef executor_manager_ref_;
};

}  // namespace aimrt::co

#else

  #include <unifex/execute.hpp>

  #include "aimrt_module_cpp_interface/executor/executor_manager.h"
  #include "aimrt_module_cpp_interface/util/function.h"

namespace aimrt::co {

// Scheduler
class AimRTScheduler {
 public:
  // OperationState
  template <typename Receiver>
  struct OperationState final {
    template <typename Receiver2>
      requires std::constructible_from<Receiver, Receiver2>
    explicit OperationState(executor::ExecutorRef executor_ref, Receiver2&& r) noexcept(
        std::is_nothrow_constructible_v<Receiver, Receiver2>)
        : executor_ref_(executor_ref), receiver_((Receiver2 &&) r) {}

    void start() noexcept {
      executor_ref_.Execute([r{std::move(receiver_)}]() mutable {
        try {
          unifex::set_value(std::move(r));
        } catch (...) {
          unifex::set_error(std::move(r), std::current_exception());
        }
      });
    }

    executor::ExecutorRef executor_ref_;
    Receiver receiver_;
  };

  // Sender
  class Task {
   public:
    template <template <typename...> class Variant,
              template <typename...> class Tuple>
    using value_types = Variant<Tuple<>>;

    template <template <typename...> class Variant>
    using error_types = Variant<std::exception_ptr>;

    static constexpr bool sends_done = false;

    explicit Task(executor::ExecutorRef executor_ref) noexcept
        : executor_ref_(executor_ref) {}

    template <typename Receiver>
    auto connect(Receiver&& receiver) {
      return OperationState<unifex::remove_cvref_t<Receiver>>(
          executor_ref_, (Receiver &&) receiver);
    }

   private:
    executor::ExecutorRef executor_ref_;
  };

  // OperationState
  template <typename Receiver>
  struct SchedulerAfterOperationState final {
    template <typename Receiver2>
      requires std::constructible_from<Receiver, Receiver2>
    explicit SchedulerAfterOperationState(
        executor::ExecutorRef executor_ref,
        const std::chrono::nanoseconds& dt,
        Receiver2&& r)  //
        noexcept(std::is_nothrow_constructible_v<Receiver, Receiver2>)
        : executor_ref_(executor_ref), dt_(dt), receiver_((Receiver2 &&) r) {}

    void start() noexcept {
      executor_ref_.ExecuteAfter(dt_, [r{std::move(receiver_)}]() mutable {
        try {
          unifex::set_value(std::move(r));
        } catch (...) {
          unifex::set_error(std::move(r), std::current_exception());
        }
      });
    }

   private:
    executor::ExecutorRef executor_ref_;
    std::chrono::nanoseconds dt_;
    Receiver receiver_;
  };

  // Sender
  class SchedulerAfterTask {
   public:
    template <template <typename...> class Variant,
              template <typename...> class Tuple>
    using value_types = Variant<Tuple<>>;

    template <template <typename...> class Variant>
    using error_types = Variant<std::exception_ptr>;

    static constexpr bool sends_done = false;

    explicit SchedulerAfterTask(
        executor::ExecutorRef executor_ref,
        const std::chrono::nanoseconds& dt) noexcept
        : executor_ref_(executor_ref), dt_(dt) {}

    template <typename Receiver>
    auto connect(Receiver&& receiver) {
      return SchedulerAfterOperationState<unifex::remove_cvref_t<Receiver>>(
          executor_ref_, dt_, (Receiver &&) receiver);
    }

   private:
    executor::ExecutorRef executor_ref_;
    std::chrono::nanoseconds dt_;
  };

  // OperationState
  template <typename Receiver>
  struct SchedulerAtOperationState final {
    template <typename Receiver2>
      requires std::constructible_from<Receiver, Receiver2>
    explicit SchedulerAtOperationState(
        executor::ExecutorRef executor_ref,
        const std::chrono::system_clock::time_point& tp,
        Receiver2&& r)  //
        noexcept(std::is_nothrow_constructible_v<Receiver, Receiver2>)
        : executor_ref_(executor_ref), tp_(tp), receiver_((Receiver2 &&) r) {}

    void start() noexcept {
      executor_ref_.ExecuteAt(tp_, [r{std::move(receiver_)}]() mutable {
        try {
          unifex::set_value(std::move(r));
        } catch (...) {
          unifex::set_error(std::move(r), std::current_exception());
        }
      });
    }

   private:
    executor::ExecutorRef executor_ref_;
    std::chrono::system_clock::time_point tp_;
    Receiver receiver_;
  };

  // Sender
  class SchedulerAtTask {
   public:
    template <template <typename...> class Variant,
              template <typename...> class Tuple>
    using value_types = Variant<Tuple<>>;

    template <template <typename...> class Variant>
    using error_types = Variant<std::exception_ptr>;

    static constexpr bool sends_done = false;

    explicit SchedulerAtTask(
        executor::ExecutorRef executor_ref,
        const std::chrono::system_clock::time_point& tp) noexcept
        : executor_ref_(executor_ref), tp_(tp) {}

    template <typename Receiver>
    auto connect(Receiver&& receiver) {
      return SchedulerAtOperationState<unifex::remove_cvref_t<Receiver>>(
          executor_ref_, tp_, (Receiver &&) receiver);
    }

   private:
    executor::ExecutorRef executor_ref_;
    std::chrono::system_clock::time_point tp_;
  };

 public:
  explicit AimRTScheduler(executor::ExecutorRef executor_ref = {}) noexcept
      : executor_ref_(executor_ref) {}

  Task schedule() const noexcept { return Task(executor_ref_); }

  SchedulerAfterTask schedule_after(const std::chrono::nanoseconds& dt) const noexcept {
    return SchedulerAfterTask(executor_ref_, dt);
  }

  SchedulerAtTask schedule_at(const std::chrono::system_clock::time_point& tp) const noexcept {
    return SchedulerAtTask(executor_ref_, tp);
  }

  friend bool operator==(AimRTScheduler a, AimRTScheduler b) noexcept {
    return a.executor_ref_.NativeHandle() == b.executor_ref_.NativeHandle();
  }

  friend bool operator!=(AimRTScheduler a, AimRTScheduler b) noexcept {
    return a.executor_ref_.NativeHandle() != b.executor_ref_.NativeHandle();
  }

  explicit operator bool() const { return static_cast<bool>(executor_ref_); }

 private:
  executor::ExecutorRef executor_ref_;
};

// Context
class AimRTContext {
 public:
  explicit AimRTContext(executor::ExecutorManagerRef executor_manager_ref = {}) noexcept
      : executor_manager_ref_(executor_manager_ref) {}

  AimRTScheduler GetScheduler(std::string_view executor_name) const {
    return AimRTScheduler(executor_manager_ref_.GetExecutor(executor_name));
  }

  explicit operator bool() const { return static_cast<bool>(executor_manager_ref_); }

 private:
  executor::ExecutorManagerRef executor_manager_ref_;
};

}  // namespace aimrt::co

#endif
