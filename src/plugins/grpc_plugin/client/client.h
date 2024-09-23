// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <list>
#include <memory>

#include <boost/asio.hpp>

#include "grpc_plugin/client/connection.h"
#include "grpc_plugin/client/options.h"
#include "util/log_util.h"

namespace aimrt::plugins::grpc_plugin::client {

using IOCtx = boost::asio::io_context;
using Strand = boost::asio::strand<boost::asio::io_context::executor_type>;

template <class T>
using Awaitable = boost::asio::awaitable<T>;

using Request = http2::Request;
using RequestPtr = std::shared_ptr<Request>;

using Response = http2::Response;
using ResponsePtr = std::shared_ptr<Response>;

class AsioHttp2Client : public std::enable_shared_from_this<AsioHttp2Client> {
 public:
  explicit AsioHttp2Client(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioHttp2Client() = default;

  AsioHttp2Client(const AsioHttp2Client&) = delete;
  AsioHttp2Client& operator=(const AsioHttp2Client&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    logger_ptr_ = logger_ptr;
  }

  void Initialize(const ClientOptions& options) {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    options_ = ClientOptions::Verify(options);
    connection_options_ptr_ = std::make_shared<ConnectionOptions>(options_);
  }

  void Start() {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kStart) == State::kInit,
        "Method can only be called when state is 'Init'.");
  }

  void Shutdown() {
    if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
      return;

    auto self = shared_from_this();
    boost::asio::dispatch(mgr_strand_, [this, self]() {
      for (auto& connection_ptr : connection_ptr_list_) {
        connection_ptr->Shutdown();
      }
      connection_ptr_list_.clear();
    });
  }

  Awaitable<ResponsePtr> HttpSendRecvCo(const RequestPtr& req_ptr,
                                        std::chrono::nanoseconds timeout = std::chrono::seconds(5)) {
    return boost::asio::co_spawn(
        mgr_strand_,
        [this, &req_ptr, timeout]() -> Awaitable<ResponsePtr> {
          AIMRT_CHECK_ERROR_THROW(
              state_.load() == State::kStart,
              "Method can only be called when state is 'Start'.");

          std::shared_ptr<Connection> connection_ptr;

          for (auto itr = connection_ptr_list_.begin(); itr != connection_ptr_list_.end();) {
            if ((*itr)->IsRunning()) {
              if ((*itr)->CheckIdleAndUse()) {
                connection_ptr = *itr;
                break;
              }
              ++itr;
            } else {
              connection_ptr_list_.erase(itr++);
            }
          }

          if (!connection_ptr) {
            connection_ptr = std::make_shared<Connection>(io_ptr_, logger_ptr_);
            connection_ptr->Initialize(connection_options_ptr_);
            connection_ptr->Start();
          }

          connection_ptr_list_.emplace_back(connection_ptr);

          co_return co_await connection_ptr->HttpSendRecvCo(req_ptr, timeout);
        },
        boost::asio::use_awaitable);
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  bool IsRunning() const { return state_.load() == State::kStart; }

 private:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  std::shared_ptr<IOCtx> io_ptr_;
  Strand mgr_strand_;

  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  ClientOptions options_;

  std::atomic<State> state_ = State::kPreInit;

  std::shared_ptr<const ConnectionOptions> connection_options_ptr_;
  std::list<std::shared_ptr<Connection>> connection_ptr_list_;
};

}  // namespace aimrt::plugins::grpc_plugin::client