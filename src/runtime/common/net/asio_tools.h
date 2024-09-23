// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>
#include <list>
#include <memory>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace aimrt::runtime::common::net {

/**
 * @brief asio执行工具
 * @note 使用时先调用RegisterSvrFunc注册子服务的启动、停止方法，
 * 然后调用Start方法异步启动，之后可以调用join方法，等待kill信号或其他异步程序里调用Stop方法结束整个服务。
 * 并不会调用asio的stop方法，只会调用注册的stop方法，等各个子服务自己停止。
 * signals_同时承担了work_guard的功能，保证没有显式Stop之前io不会退出。
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
   * @brief 注册svr的start方法
   * @note 越早注册的start func越早执行
   * @param[in] start_func
   */
  void RegisterSvrStartFunc(std::function<void()>&& start_func) {
    if (state_.load() != State::kPreStart) [[unlikely]]
      throw std::runtime_error("Method can only be called when state is 'PreStart'.");

    start_func_vec_.emplace_back(std::move(start_func));
  }

  /**
   * @brief 注册svr的stop方法
   * @note 越早注册的stop func越晚执行
   * @param[in] stop_func
   */
  void RegisterSvrStopFunc(std::function<void()>&& stop_func) {
    if (state_.load() != State::kPreStart) [[unlikely]]
      throw std::runtime_error("Method can only be called when state is 'PreStart'.");

    stop_func_vec_.emplace_back(std::move(stop_func));
  }

  /**
   * @brief 注册svr的start、stop方法
   * @note 越早注册的start func越早执行，越早注册的stop func越晚执行
   * @param[in] start_func 子服务启动方法，一般在其中起一个启动协程
   * @param[in] stop_func 子服务结束方法，需要保证可以重复调用
   */
  void RegisterSvrFunc(std::function<void()>&& start_func, std::function<void()>&& stop_func) {
    RegisterSvrStartFunc(std::move(start_func));
    RegisterSvrStopFunc(std::move(stop_func));
  }

  /**
   * @brief 开始运行
   * @note 异步，会调用注册的start方法并启动指定数量的线程
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
   * @brief join
   * @note 阻塞直到所有线程退出
   */
  void Join() {
    for (auto itr = threads_.begin(); itr != threads_.end();) {
      if (itr->joinable()) itr->join();
      threads_.erase(itr++);
    }
  }

  /**
   * @brief 停止
   * @note 异步，会调用注册的stop方法
   */
  void Shutdown() {
    if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown) [[unlikely]]
      return;

    // 并不需要调用io_.stop()。当io_上所有任务都运行完毕后，会自动停止
    std::for_each(stop_func_vec_.rbegin(), stop_func_vec_.rend(),
                  [](const std::function<void()>& f) {
                    if (f) f();
                  });

    stop_func_vec_.clear();

    work_guard_.reset();
  }

  /**
   * @brief 接收停止信号
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

  /**
   * @brief 获取io_ctx
   * @return io_context
   */
  auto IO() const { return io_ptr_; }

  /**
   * @brief 获取线程数
   * @return uint32_t
   */
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

}  // namespace aimrt::runtime::common::net
