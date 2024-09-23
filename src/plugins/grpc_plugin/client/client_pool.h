// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>

#include <boost/asio.hpp>

#include "grpc_plugin/client/client.h"
#include "grpc_plugin/client/options.h"
#include "util/log_util.h"

namespace aimrt::plugins::grpc_plugin::client {

using IOCtx = boost::asio::io_context;
using Strand = boost::asio::strand<boost::asio::io_context::executor_type>;

template <class T>
using Awaitable = boost::asio::awaitable<T>;

class AsioHttp2ClientPool
    : public std::enable_shared_from_this<AsioHttp2ClientPool> {
 public:
  explicit AsioHttp2ClientPool(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioHttp2ClientPool() = default;

  AsioHttp2ClientPool(const AsioHttp2ClientPool&) = delete;
  AsioHttp2ClientPool& operator=(const AsioHttp2ClientPool&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    logger_ptr_ = logger_ptr;
  }

  void Initialize(const ClientPoolOptions& options) {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    options_ = ClientPoolOptions::Verify(options);
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
      for (auto& itr : client_map_) {
        itr.second->Shutdown();
      }

      client_map_.clear();
    });
  }

  Awaitable<std::shared_ptr<AsioHttp2Client>> GetClient(
      const ClientOptions& client_options) {
    return boost::asio::co_spawn(
        mgr_strand_,
        [this, &client_options]() -> Awaitable<std::shared_ptr<AsioHttp2Client>> {
          if (state_.load() != State::kStart) [[unlikely]] {
            AIMRT_WARN("Method can only be called when state is 'Start'.");
            co_return std::shared_ptr<AsioHttp2Client>();
          }

          auto client_key = client_options.host + client_options.service;

          auto itr = client_map_.find(client_key);
          if (itr != client_map_.end()) {
            if (itr->second->IsRunning()) co_return itr->second;
            client_map_.erase(itr);
          }

          if (client_map_.size() >= options_.max_client_num) [[unlikely]] {
            for (auto itr = client_map_.begin(); itr != client_map_.end();) {
              if (itr->second->IsRunning())
                ++itr;
              else
                client_map_.erase(itr++);
            }

            AIMRT_CHECK_ERROR_THROW(client_map_.size() < options_.max_client_num,
                                    "Client pool is full.");
          }

          auto client_ptr = std::make_shared<AsioHttp2Client>(io_ptr_);
          client_ptr->SetLogger(logger_ptr_);
          client_ptr->Initialize(client_options);
          client_ptr->Start();
          client_map_.emplace(client_key, client_ptr);
          co_return client_ptr;
        },
        boost::asio::use_awaitable);
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

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

  ClientPoolOptions options_;

  std::atomic<State> state_ = State::kPreInit;

  // Client map, the key is the client's host and service
  std::unordered_map<std::string, std::shared_ptr<AsioHttp2Client>> client_map_;
};

}  // namespace aimrt::plugins::grpc_plugin::client