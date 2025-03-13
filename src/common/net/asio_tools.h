// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>
#include <list>
#include <memory>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace aimrt::common::net {

/**
 * @brief Asio Executor Tool
 * @note When using, first call RegisterSvrFunc to register the start and stop methods of the subservice.
 * Then call the Start method to start asynchronously.
 * And then call the join method to wait for the kill signal or other asynchronous programs to call the Stop method to end the entire service.
 * It will not call the asio stop method, but only call the registered stop method, and wait for each subservice to stop by itself.
 * 'signals_' also assumes the function of work_guard, ensuring that io will not exit before there is an explicit Stop.
 */
class AsioExecutor {
 public:
  explicit AsioExecutor(uint32_t threads_num = std::max<uint32_t>(std::thread::hardware_concurrency(), 1))
      : threads_num_(threads_num),
        io_ptr_(std::make_shared<boost::asio::io_context>(threads_num)),
        work_guard_(io_ptr_->get_executor()) {
  }

  ~AsioExecutor() {
    try {
      Shutdown();
      Join();
    } catch (const std::exception& e) {
      fprintf(stderr, "AsioExecutor destruct get exception, %s\n", e.what());
    }
  }

  AsioExecutor(const AsioExecutor&) = delete;
  AsioExecutor& operator=(const AsioExecutor&) = delete;

  /**
   * @brief Register svr start method
   * @note The earlier the start func is registered, the earlier it will be executed.
   * @param[in] start_func
   */
  void RegisterSvrStartFunc(std::function<void()>&& start_func) {
    if (state_.load() != State::kPreStart) [[unlikely]]
      throw std::runtime_error("Method can only be called when state is 'PreStart'.");

    start_func_vec_.emplace_back(std::move(start_func));
  }

  /**
   * @brief Register svr stop method
   * @note The earlier the stop func is registered, the later it will be executed.
   * @param[in] stop_func
   */
  void RegisterSvrStopFunc(std::function<void()>&& stop_func) {
    if (state_.load() != State::kPreStart) [[unlikely]]
      throw std::runtime_error("Method can only be called when state is 'PreStart'.");

    stop_func_vec_.emplace_back(std::move(stop_func));
  }

  /**
   * @brief Register svr start/stop method
   * @note The earlier the start func is registered, the earlier it is executed. The earlier the stop func is registered, the later it is executed.
   * @param[in] start_func
   * @param[in] stop_func
   */
  void RegisterSvrFunc(std::function<void()>&& start_func, std::function<void()>&& stop_func) {
    RegisterSvrStartFunc(std::move(start_func));
    RegisterSvrStopFunc(std::move(stop_func));
  }

  /**
   * @brief Start running
   * @note Asynchronously, the registered start method will be called and the specified number of threads will be started.
   */
  void Start() {
    if (std::atomic_exchange(&state_, State::kStart) != State::kPreStart) [[unlikely]]
      throw std::runtime_error("Method can only be called when state is 'PreStart'.");

    std::for_each(start_func_vec_.begin(), start_func_vec_.end(),
                  [](const std::function<void()>& f) {
                    if (f) f();
                  });

    start_func_vec_.clear();

    auto run_func = [this] {
      try {
        io_ptr_->run();
      } catch (const std::exception& e) {
        fprintf(stderr, "AsioExecutor thread get exception %s.\n", e.what());
      }
    };

    for (uint32_t ii = 0; ii < threads_num_; ++ii) {
      threads_.emplace_back(run_func);
    }
  }

  /**
   * @brief Join
   * @note Block until all threads exit
   */
  void Join() {
    for (auto itr = threads_.begin(); itr != threads_.end();) {
      if (itr->joinable()) itr->join();
      threads_.erase(itr++);
    }
  }

  /**
   * @brief Shutdown
   * @note Asynchronous, the registered stop method will be called
   */
  void Shutdown() {
    if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown) [[unlikely]]
      return;

    // There is no need to call 'io_.stop()'. When all tasks on io_ are completed, it will stop automatically.
    std::for_each(stop_func_vec_.rbegin(), stop_func_vec_.rend(),
                  [](const std::function<void()>& f) {
                    if (f) f();
                  });

    stop_func_vec_.clear();

    work_guard_.reset();
  }

  /**
   * @brief Allow receiving a stop signal
   *
   */
  void EnableStopSignal() {
    if (state_.load() != State::kPreStart) [[unlikely]]
      throw std::runtime_error("Method can only be called when state is 'PreStart'.");

    auto sig_ptr = std::make_shared<boost::asio::signal_set>(*io_ptr_, SIGINT, SIGTERM);

    start_func_vec_.emplace_back([this, sig_ptr] {
      sig_ptr->async_wait([this, sig_ptr](auto, auto) {
        Shutdown();
      });
    });

    stop_func_vec_.emplace_back([sig_ptr] {
      sig_ptr->cancel();
      sig_ptr->clear();
    });
  }

  auto IO() const { return io_ptr_; }

  uint32_t ThreadsNum() const { return threads_num_; }

 private:
  enum class State : uint32_t {
    kPreStart,
    kStart,
    kShutdown,
  };

  const uint32_t threads_num_;

  std::atomic<State> state_ = State::kPreStart;

  std::shared_ptr<boost::asio::io_context> io_ptr_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  std::list<std::thread> threads_;
  std::vector<std::function<void()>> start_func_vec_;
  std::vector<std::function<void()>> stop_func_vec_;
};

}  // namespace aimrt::common::net
