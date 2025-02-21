// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "time_manipulator_plugin/time_manipulator_executor.h"
#include "core/util/thread_tools.h"
#include "time_manipulator_plugin/global.h"
#include "util/time_util.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::time_manipulator_plugin::TimeManipulatorExecutor::Options> {
  using Options = aimrt::plugins::time_manipulator_plugin::TimeManipulatorExecutor::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["bind_executor"] = rhs.bind_executor;
    node["dt_us"] = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(rhs.dt).count());
    node["init_ratio"] = rhs.init_ratio;
    node["wheel_size"] = rhs.wheel_size;
    node["thread_sched_policy"] = rhs.thread_sched_policy;
    node["thread_bind_cpu"] = rhs.thread_bind_cpu;
    node["use_system_clock"] = rhs.use_system_clock;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.bind_executor = node["bind_executor"].as<std::string>();

    if (node["dt_us"])
      rhs.dt = std::chrono::microseconds(node["dt_us"].as<uint64_t>());
    if (node["init_ratio"])
      rhs.init_ratio = node["init_ratio"].as<double>();
    if (node["wheel_size"])
      rhs.wheel_size = node["wheel_size"].as<std::vector<size_t>>();
    if (node["thread_sched_policy"])
      rhs.thread_sched_policy = node["thread_sched_policy"].as<std::string>();
    if (node["thread_bind_cpu"])
      rhs.thread_bind_cpu = node["thread_bind_cpu"].as<std::vector<uint32_t>>();
    if (node["use_system_clock"])
      rhs.use_system_clock = node["use_system_clock"].as<bool>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::time_manipulator_plugin {

void TimeManipulatorExecutor::Initialize(std::string_view name,
                                         YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      get_executor_func_,
      "Get executor function is not set before initialize.");

  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "TimeManipulatorExecutor can only be initialized once.");

  name_ = std::string(name);

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  AIMRT_CHECK_ERROR_THROW(
      !options_.bind_executor.empty(),
      "Invalide bind executor name, name is empty.");

  bind_executor_ref_ = get_executor_func_(options_.bind_executor);

  AIMRT_CHECK_ERROR_THROW(
      bind_executor_ref_,
      "Can not get executor {}.", options_.bind_executor);

  thread_safe_ = bind_executor_ref_.ThreadSafe();

  SetTimeRatio(options_.init_ratio);

  dt_count_ = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(options_.dt).count());

  uint64_t cur_scale = 1;
  for (size_t ii = 0; ii < options_.wheel_size.size(); ++ii) {
    uint64_t start_pos = (ii == 0) ? 0 : 1;
    timing_wheel_vec_.emplace_back(TimingWheelTool{
        .current_pos = start_pos,
        .scale = (cur_scale *= options_.wheel_size[ii]),
        .wheel = std::vector<TaskList>(options_.wheel_size[ii]),
        .borrow_func = [ii, this]() {
          TaskList task_list;
          if (ii < options_.wheel_size.size() - 1) {
            task_list = timing_wheel_vec_[ii + 1].Tick();
          } else {
            auto itr = timing_task_map_.find(timing_task_map_pos_);
            ++timing_task_map_pos_;
            if (itr != timing_task_map_.end()) {
              task_list = std::move(itr->second);
              timing_task_map_.erase(itr);
            }
          }

          while (!task_list.empty()) {
            auto itr = task_list.begin();
            auto& cur_list = timing_wheel_vec_[ii].wheel[itr->tick_count % timing_wheel_vec_[ii].scale];
            cur_list.splice(cur_list.end(), task_list, itr);
          }
        }});
  }

  timing_task_map_pos_ = 1;

  options_node = options_;
}

void TimeManipulatorExecutor::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  timer_thread_ptr_ = std::make_unique<std::thread>(std::bind(&TimeManipulatorExecutor::TimerLoop, this));

  start_flag_.wait(false);
}

void TimeManipulatorExecutor::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  if (timer_thread_ptr_ && timer_thread_ptr_->joinable())
    timer_thread_ptr_->join();

  timer_thread_ptr_.reset();
  timing_task_map_.clear();
  timing_wheel_vec_.clear();
  get_executor_func_ = std::function<executor::ExecutorRef(std::string_view)>();
}

bool TimeManipulatorExecutor::IsInCurrentExecutor() const noexcept {
  try {
    return bind_executor_ref_.IsInCurrentExecutor();
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
  return false;
}

void TimeManipulatorExecutor::Execute(aimrt::executor::Task&& task) noexcept {
  try {
    bind_executor_ref_.Execute(std::move(task));
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

std::chrono::system_clock::time_point TimeManipulatorExecutor::Now() const noexcept {
  std::shared_lock<std::shared_mutex> lck(tick_mutex_);

  return aimrt::common::util::GetTimePointFromTimestampNs(
      current_tick_count_ * dt_count_ + start_time_point_);
}

void TimeManipulatorExecutor::ExecuteAt(
    std::chrono::system_clock::time_point tp, aimrt::executor::Task&& task) noexcept {
  try {
    uint64_t virtual_tp = aimrt::common::util::GetTimestampNs(tp) - start_time_point_;

    std::unique_lock<std::shared_mutex> lck(tick_mutex_);

    if (virtual_tp < current_tick_count_ * dt_count_) {
      lck.unlock();
      bind_executor_ref_.Execute(std::move(task));
      return;
    }

    // 当前时间点 time_point_
    uint64_t temp_current_tick_count = current_tick_count_;
    uint64_t diff_tick_count = virtual_tp / dt_count_ - current_tick_count_;

    const size_t len = options_.wheel_size.size();
    for (size_t ii = 0; ii < len; ++ii) {
      if (diff_tick_count < options_.wheel_size[ii]) {
        auto pos = (diff_tick_count + temp_current_tick_count) % options_.wheel_size[ii];

        // TODO：基于时间将任务排序后插进去
        timing_wheel_vec_[ii].wheel[pos].emplace_back(
            TaskWithTimestamp{virtual_tp / dt_count_, std::move(task)});
        return;
      }
      diff_tick_count /= options_.wheel_size[ii];
      temp_current_tick_count /= options_.wheel_size[ii];
    }

    timing_task_map_[diff_tick_count + temp_current_tick_count].emplace_back(
        TaskWithTimestamp{virtual_tp / dt_count_, std::move(task)});
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void TimeManipulatorExecutor::RegisterGetExecutorFunc(
    const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  get_executor_func_ = get_executor_func;
}

void TimeManipulatorExecutor::SetTimeRatio(double ratio) {
  std::unique_lock<std::shared_mutex> lck(ratio_mutex_);

  // 大于1，快进
  if (ratio >= 1.0) {
    ratio_direction_ = true;
    real_ratio_ = static_cast<uint32_t>(ratio);
    return;
  }

  ratio_direction_ = false;

  // 大于0小于1，慢放
  if (ratio > 1e-15 && ratio < 1.0 &&
      (1.0 / ratio) < std::numeric_limits<uint32_t>::max()) {
    real_ratio_ = static_cast<uint32_t>(1.0 / ratio);
    return;
  }

  // 小于0等于0，暂停
  real_ratio_ = std::numeric_limits<uint32_t>::max();
}

double TimeManipulatorExecutor::GetTimeRatio() const {
  std::shared_lock<std::shared_mutex> lck(ratio_mutex_);

  if (ratio_direction_)
    return real_ratio_;

  if (real_ratio_ == std::numeric_limits<uint32_t>::max())
    return 0.0;

  return 1.0 / real_ratio_;
}

void TimeManipulatorExecutor::TimerLoop() {
  std::string threadname = name_;

  try {
    aimrt::runtime::core::util::SetNameForCurrentThread(threadname);
    aimrt::runtime::core::util::BindCpuForCurrentThread(options_.thread_bind_cpu);
    aimrt::runtime::core::util::SetCpuSchedForCurrentThread(options_.thread_sched_policy);
  } catch (const std::exception& e) {
    AIMRT_WARN("Set thread policy for time manipulator executor '{}' get exception, {}",
               Name(), e.what());
  }

  auto last_loop_sys_tp = std::chrono::system_clock::now();
  auto last_loop_std_tp = std::chrono::steady_clock::now();

  // 记录初始时间
  start_time_point_ =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          (options_.use_system_clock ? last_loop_sys_tp.time_since_epoch() : last_loop_std_tp.time_since_epoch()))
          .count();

  start_flag_.store(true);
  start_flag_.notify_all();

  while (state_.load() != State::kShutdown) {
    try {
      // 获取时间比例。注意：调速只能在下一个tick生效
      ratio_mutex_.lock_shared();

      bool ratio_direction = ratio_direction_;
      uint32_t real_ratio = real_ratio_;

      ratio_mutex_.unlock_shared();

      // sleep一个tick
      if (real_ratio == std::numeric_limits<uint32_t>::max()) {
        // 暂停了
        if (!options_.use_system_clock) {
          std::this_thread::sleep_until(last_loop_std_tp += std::chrono::seconds(1));
        } else {
          std::this_thread::sleep_until(last_loop_sys_tp += std::chrono::seconds(1));
        }

        continue;
      }

      auto real_dt = ratio_direction ? options_.dt : (options_.dt * real_ratio);
      do {
        // 最长sleep时间
        static constexpr auto kMaxSleepDt = std::chrono::seconds(1);

        auto sleep_time = (real_dt > kMaxSleepDt) ? kMaxSleepDt : real_dt;
        real_dt -= sleep_time;

        // 一个小优化，防止real_dt太小
        if (real_dt.count() && options_.dt < kMaxSleepDt && real_dt <= options_.dt) {
          sleep_time += real_dt;
          real_dt -= real_dt;
        }

        if (!options_.use_system_clock) {
          std::this_thread::sleep_until(
              last_loop_std_tp +=
              std::chrono::duration_cast<std::chrono::steady_clock::time_point::duration>(sleep_time));
        } else {
          std::this_thread::sleep_until(
              last_loop_sys_tp +=
              std::chrono::duration_cast<std::chrono::system_clock::time_point::duration>(sleep_time));
        }

      } while (state_.load() != State::kShutdown && real_dt.count());

      // 走时间轮

      // 要走的tick数
      uint64_t diff_tick_count = ratio_direction ? real_ratio : 1;

      tick_mutex_.lock();

      do {
        // 取出task
        TaskList task_list = timing_wheel_vec_[0].Tick();

        // 执行任务
        if (!task_list.empty()) {
          tick_mutex_.unlock();

          for (auto& itr : task_list) {
            bind_executor_ref_.Execute(std::move(itr.task));
          }

          tick_mutex_.lock();
        }

        // 更新time point
        ++current_tick_count_;

      } while (--diff_tick_count);

      tick_mutex_.unlock();
    } catch (const std::exception& e) {
      AIMRT_FATAL("Time manipulator executor '{}' run loop get exception, {}",
                  Name(), e.what());
    }
  }
}
}  // namespace aimrt::plugins::time_manipulator_plugin
