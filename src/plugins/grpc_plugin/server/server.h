// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <cstdint>
#include <list>
#include <memory>
#include <string_view>

#include <boost/asio/awaitable.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/use_awaitable.hpp>

#include "grpc_plugin/server/connection.h"
#include "grpc_plugin/server/options.h"
#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::plugins::grpc_plugin::server {

class AsioHttp2Server : public std::enable_shared_from_this<AsioHttp2Server> {
 public:
  explicit AsioHttp2Server(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr)),
        acceptor_(mgr_strand_),
        acceptor_timer_(mgr_strand_),
        mgr_timer_(mgr_strand_),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()),
        dispatcher_ptr_(std::make_shared<Dispatcher>()) {}

  ~AsioHttp2Server() = default;

  AsioHttp2Server(const AsioHttp2Server&) = delete;
  AsioHttp2Server& operator=(const AsioHttp2Server&) = delete;

  const aimrt::common::util::LoggerWrapper& GetLogger() const noexcept { return *logger_ptr_; }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    logger_ptr_ = logger_ptr;
  }

  void RegisterHttpHandleFunc(std::string_view pattern, HttpHandle handle) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    dispatcher_ptr_->RegisterHttpHandle(pattern, std::move(handle));
  }

  void Initialize(const ServerOptions& options) {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    options_ = ServerOptions::Verify(options);
    connection_options_ptr_ = std::make_shared<ConnectionOptions>(options_);

    AIMRT_CHECK_ERROR_THROW(CheckListenAddr(options_.ep),
                            "{} is already in use.", aimrt::common::util::SSToString(options_.ep));
  }

  void Start() {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kStart) == State::kInit,
        "Method can only be called when state is 'Init'.");

    auto self = this->shared_from_this();
    boost::asio::co_spawn(
        mgr_strand_,
        [this, self]() -> Awaitable<void> {
          acceptor_.open(options_.ep.protocol());
          acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
          acceptor_.bind(options_.ep);
          acceptor_.listen();

          while (state_.load() == State::kStart) {
            try {
              if (connection_ptr_list_.size() >= options_.max_connection_num) {
                acceptor_timer_.expires_after(options_.mgr_timer_dt);
                co_await acceptor_timer_.async_wait(boost::asio::use_awaitable);
                continue;
              }

              auto connection_ptr = std::make_shared<Connection>(
                  io_ptr_, logger_ptr_, dispatcher_ptr_);
              connection_ptr->Initialize(connection_options_ptr_);

              co_await acceptor_.async_accept(connection_ptr->Socket(), boost::asio::use_awaitable);
              AIMRT_TRACE("Http svr accept a new connection, remote addr is {}",
                          aimrt::common::util::SSToString(connection_ptr->Socket().remote_endpoint()));
              connection_ptr->Start();

              connection_ptr_list_.emplace_back(connection_ptr);
            } catch (const std::exception& e) {
              AIMRT_WARN("Http svr accept connection get exception and exit, exception info: {}", e.what());
            }
          }

          Shutdown();

          co_return;
        },
        boost::asio::detached);

    boost::asio::co_spawn(
        mgr_strand_,
        [this, self]() -> Awaitable<void> {
          while (state_.load() == State::kStart) {
            try {
              mgr_timer_.expires_after(options_.mgr_timer_dt);
              co_await mgr_timer_.async_wait(boost::asio::use_awaitable);

              for (auto itr = connection_ptr_list_.begin(); itr != connection_ptr_list_.end();) {
                if ((*itr)->IsRunning())
                  ++itr;
                else
                  connection_ptr_list_.erase(itr++);
              }
            } catch (const std::exception& e) {
              AIMRT_WARN(
                  "Http svr timer get exception and exit, exception info: {}",
                  e.what());
            }
          }

          Shutdown();

          co_return;
        },
        boost::asio::detached);
  }

  void Shutdown() {
    if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown) {
      return;
    }

    auto self = this->shared_from_this();
    boost::asio::dispatch(mgr_strand_, [this, self]() {
      try {
        acceptor_.close();
      } catch (const std::exception& e) {
        AIMRT_TRACE("Http svr shutdown failed, exception info: {}", e.what());
      }
    });

    boost::asio::dispatch(mgr_strand_, [this, self]() {
      try {
        acceptor_timer_.cancel();
        mgr_timer_.cancel();
        acceptor_.cancel();
        acceptor_.close();
        acceptor_.release();
      } catch (const std::exception& e) {
        AIMRT_TRACE("Http svr mgr shutdown failed, exception info: {}", e.what());
      }
    });

    for (auto& connection_ptr : connection_ptr_list_) {
      connection_ptr->Shutdown();
    }

    connection_ptr_list_.clear();
  }

 private:
  static bool CheckListenAddr(const boost::asio::ip::tcp::endpoint& ep) {
    try {
      IOCtx io;
      Tcp::acceptor acceptor(io, ep);
      return true;
    } catch (...) {
      return false;
    }
  }

 private:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  std::shared_ptr<IOCtx> io_ptr_;
  Strand mgr_strand_;
  Tcp::acceptor acceptor_;
  Timer acceptor_timer_;
  Timer mgr_timer_;

  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::shared_ptr<Dispatcher> dispatcher_ptr_;
  ServerOptions options_;

  std::atomic<State> state_ = State::kPreInit;

  std::shared_ptr<ConnectionOptions> connection_options_ptr_;
  std::list<std::shared_ptr<Connection>> connection_ptr_list_;
};

}  // namespace aimrt::plugins::grpc_plugin::server