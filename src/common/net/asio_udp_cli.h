// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <chrono>
#include <list>
#include <memory>
#include <unordered_map>

#include <boost/asio.hpp>
#include <utility>

#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::common::net {

class AsioUdpClient : public std::enable_shared_from_this<AsioUdpClient> {
 public:
  using IOCtx = boost::asio::io_context;
  using Udp = boost::asio::ip::udp;
  using Strand = boost::asio::strand<IOCtx::executor_type>;
  using Timer = boost::asio::steady_timer;
  using Streambuf = boost::asio::streambuf;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  struct Options {
    /// Server address
    Udp::endpoint svr_ep;

    // Maximum time without data
    std::chrono::nanoseconds max_no_data_duration = std::chrono::seconds(60);

    // The maximum length of each packet. Cannot be greater than 65507
    size_t max_package_size = 65507;

    static Options Verify(const Options& verify_options) {
      Options options(verify_options);

      if (options.max_package_size > 65507) options.max_package_size = 65507;

      return options;
    }
  };

  explicit AsioUdpClient(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioUdpClient() = default;

  AsioUdpClient(const AsioUdpClient&) = delete;
  AsioUdpClient& operator=(const AsioUdpClient&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    logger_ptr_ = logger_ptr;
  }

  void Initialize(const Options& options) {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    options_ = Options::Verify(options);
    session_options_ptr_ = std::make_shared<SessionOptions>(options_);
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
      if (session_ptr_) {
        session_ptr_->Shutdown();
        session_ptr_.reset();
      }
    });
  }

  void SendMsg(const std::shared_ptr<Streambuf>& msg_buf_ptr) {
    auto self = shared_from_this();
    boost::asio::dispatch(mgr_strand_, [this, self, msg_buf_ptr]() {
      if (state_.load() != State::kStart) [[unlikely]] {
        AIMRT_WARN("Udp cli is closed, will not send current msg.");
        return;
      }

      AIMRT_CHECK_ERROR(msg_buf_ptr->size() <= options_.max_package_size,
                        "Msg too large for udp package, size {}, limit {}.",
                        msg_buf_ptr->size(), options_.max_package_size);

      if (!session_ptr_ || !session_ptr_->IsRunning()) {
        session_ptr_ = std::make_shared<Session>(io_ptr_, logger_ptr_);
        session_ptr_->Initialize(session_options_ptr_);
        session_ptr_->Start();
      }

      session_ptr_->SendMsg(msg_buf_ptr);
    });
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  bool IsRunning() const { return state_.load() == State::kStart; }

 private:
  struct SessionOptions {
    explicit SessionOptions(const Options& options)
        : svr_ep(options.svr_ep),
          max_no_data_duration(options.max_no_data_duration) {}

    Udp::endpoint svr_ep;
    std::chrono::nanoseconds max_no_data_duration;
  };

  class Session : public std::enable_shared_from_this<Session> {
   public:
    Session(const std::shared_ptr<IOCtx>& io_ptr,
            const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr)
        : io_ptr_(io_ptr),
          session_mgr_strand_(boost::asio::make_strand(*io_ptr)),
          timer_(session_mgr_strand_),
          session_socket_strand_(boost::asio::make_strand(*io_ptr)),
          sock_(session_socket_strand_),
          logger_ptr_(logger_ptr) {}

    ~Session() = default;

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    void Initialize(const std::shared_ptr<const SessionOptions>& session_options_ptr) {
      AIMRT_CHECK_ERROR_THROW(
          std::atomic_exchange(&state_, SessionState::kInit) == SessionState::kPreInit,
          "Method can only be called when state is 'PreInit'.");

      session_options_ptr_ = session_options_ptr;
    }

    void Start() {
      AIMRT_CHECK_ERROR_THROW(
          std::atomic_exchange(&state_, SessionState::kStart) == SessionState::kInit,
          "Method can only be called when state is 'Init'.");

      sock_.open(boost::asio::ip::udp::v4());

      auto self = shared_from_this();

      // 超时检查协程
      boost::asio::co_spawn(
          session_mgr_strand_,
          [this, self]() -> Awaitable<void> {
            try {
              while (state_.load() == SessionState::kStart) {
                timer_.expires_after(session_options_ptr_->max_no_data_duration);
                co_await timer_.async_wait(boost::asio::use_awaitable);

                if (tick_has_data_) {
                  tick_has_data_ = false;
                } else {
                  AIMRT_TRACE(
                      "udp cli session exit due to timeout({}ms).",
                      std::chrono::duration_cast<std::chrono::milliseconds>(session_options_ptr_->max_no_data_duration).count());
                  break;
                }
              }
            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "udp cli session timer get exception and exit, exception info: {}",
                  e.what());
            }

            Shutdown();

            co_return;
          },
          boost::asio::detached);
    }

    void Shutdown() {
      if (std::atomic_exchange(&state_, SessionState::kShutdown) == SessionState::kShutdown)
        return;

      auto self = shared_from_this();
      boost::asio::dispatch(session_socket_strand_, [this, self]() {
        uint32_t stop_step = 1;
        while (stop_step) {
          try {
            switch (stop_step) {
              case 1:
                sock_.shutdown(boost::asio::ip::udp::socket::shutdown_both);
                ++stop_step;
              case 2:
                sock_.cancel();
                ++stop_step;
              case 3:
                sock_.close();
                ++stop_step;
              case 4:
                sock_.release();
                ++stop_step;
              default:
                stop_step = 0;
                break;
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE(
                "udp cli session stop get exception at step {}, exception info: {}",
                stop_step, e.what());
            ++stop_step;
          }
        }
      });

      boost::asio::dispatch(session_mgr_strand_, [this, self]() {
        uint32_t stop_step = 1;
        while (stop_step) {
          try {
            switch (stop_step) {
              case 1:
                timer_.cancel();
                ++stop_step;
              default:
                stop_step = 0;
                break;
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE(
                "udp cli session mgr stop get exception at step {}, exception info: {}",
                stop_step, e.what());
            ++stop_step;
          }
        }
      });
    }

    void SendMsg(const std::shared_ptr<Streambuf>& msg_buf_ptr) {
      auto self = shared_from_this();
      boost::asio::co_spawn(
          session_socket_strand_,
          [this, self, msg_buf_ptr]() -> Awaitable<void> {
            if (state_.load() != SessionState::kStart) [[unlikely]] {
              AIMRT_WARN("Udp cli session is closed, will not send current msg.");
              co_return;
            }

            try {
              size_t data_size = msg_buf_ptr->size();
              size_t send_data_size = co_await sock_.async_send_to(
                  msg_buf_ptr->data(), session_options_ptr_->svr_ep,
                  boost::asio::use_awaitable);
              AIMRT_TRACE("udp cli session async write {} bytes to {}",
                          send_data_size, aimrt::common::util::SSToString(session_options_ptr_->svr_ep));
              if (send_data_size != data_size) {
                AIMRT_WARN(
                    "udp send msg incomplete, expected send data size {}, actual send data size {}",
                    data_size, send_data_size);
              }

              tick_has_data_ = true;
            } catch (const std::exception& e) {
              AIMRT_WARN("udp send msg get exception, exception info: {}",
                         e.what());
            }
            co_return;
          },
          boost::asio::detached);
    }

    const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

    bool IsRunning() const { return state_.load() == SessionState::kStart; }

   private:
    enum class SessionState : uint32_t {
      kPreInit,
      kInit,
      kStart,
      kShutdown,
    };

    // IO CTX
    std::shared_ptr<IOCtx> io_ptr_;
    Strand session_mgr_strand_;
    Timer timer_;
    Strand session_socket_strand_;
    Udp::socket sock_;

    // Log handle
    std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

    // Options
    std::shared_ptr<const SessionOptions> session_options_ptr_;

    // State
    std::atomic<SessionState> state_ = SessionState::kPreInit;

    // misc
    std::atomic_bool tick_has_data_ = false;
  };

 private:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  // IO CTX
  std::shared_ptr<IOCtx> io_ptr_;
  Strand mgr_strand_;

  // Log handle
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  // Options
  Options options_;

  // State
  std::atomic<State> state_ = State::kPreInit;

  // Session management
  std::shared_ptr<const SessionOptions> session_options_ptr_;
  std::shared_ptr<Session> session_ptr_;
};

class AsioUdpClientPool
    : public std::enable_shared_from_this<AsioUdpClientPool> {
 public:
  using IOCtx = boost::asio::io_context;
  using Strand = boost::asio::strand<IOCtx::executor_type>;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  struct Options {
    /// Maximum number of clients
    size_t max_client_num = 1000;

    static Options Verify(const Options& verify_options) {
      Options options(verify_options);

      if (options.max_client_num < 10) options.max_client_num = 10;

      return options;
    }
  };

  explicit AsioUdpClientPool(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioUdpClientPool() = default;

  AsioUdpClientPool(const AsioUdpClientPool&) = delete;
  AsioUdpClientPool& operator=(const AsioUdpClientPool&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    logger_ptr_ = logger_ptr;
  }

  void Initialize(const Options& options) {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    options_ = Options::Verify(options);
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
      for (auto& itr : client_map_) itr.second->Shutdown();

      client_map_.clear();
    });
  }

  /**
   * @brief 获取udp client
   * @note 如果udp client目的地址相同，则会复用已有的udp client
   * @param cfg udp client的配置
   * @return udp client
   */
  Awaitable<std::shared_ptr<AsioUdpClient>> GetClient(
      const AsioUdpClient::Options& client_options) {
    return boost::asio::co_spawn(
        mgr_strand_,
        [this, &client_options]() -> Awaitable<std::shared_ptr<AsioUdpClient>> {
          if (state_.load() != State::kStart) [[unlikely]] {
            AIMRT_WARN("Udp cli pool is closed, will not return cli instance.");
            co_return std::shared_ptr<AsioUdpClient>();
          }

          const size_t client_hash =
              std::hash<boost::asio::ip::udp::endpoint>{}(client_options.svr_ep);

          auto itr = client_map_.find(client_hash);
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

            AIMRT_CHECK_WARN_THROW(client_map_.size() < options_.max_client_num,
                                   "Udp client num reach the upper limit.");
          }

          auto client_ptr = std::make_shared<AsioUdpClient>(io_ptr_);
          client_ptr->SetLogger(logger_ptr_);
          client_ptr->Initialize(client_options);
          client_ptr->Start();

          client_map_.emplace(client_hash, client_ptr);
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

  // IO CTX
  std::shared_ptr<IOCtx> io_ptr_;
  Strand mgr_strand_;

  // Log handle
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  // Options
  Options options_;

  // State
  std::atomic<State> state_ = State::kPreInit;

  // Client management
  std::unordered_map<size_t, std::shared_ptr<AsioUdpClient>> client_map_;
};

}  // namespace aimrt::common::net
