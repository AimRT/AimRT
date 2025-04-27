// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <unordered_map>

#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::common::net {

class AsioWebSocketClient
    : public std::enable_shared_from_this<AsioWebSocketClient> {
 public:
  using IOCtx = boost::asio::io_context;
  using Tcp = boost::asio::ip::tcp;
  using WsStream = boost::beast::websocket::stream<boost::beast::tcp_stream>;
  using Strand = boost::asio::strand<IOCtx::executor_type>;
  using Timer = boost::asio::steady_timer;
  using Streambuf = boost::asio::streambuf;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  using MsgHandle = std::function<void(const std::shared_ptr<Streambuf>&)>;

  struct Options {
    /// Server domain name or IP address
    std::string host;

    /// Service (such as HTTP, FTP) or port number
    std::string service;

    /// Server path, starting with '/'
    std::string path = "/";

    /// Timer Interval
    std::chrono::nanoseconds heart_beat_time = std::chrono::seconds(60);

    /// Maximum package size: up to 10m
    uint32_t max_recv_size = 1024 * 1024 * 10;

    // Use binary mode or text mode
    bool binary_mode = true;

    static Options Verify(const Options& verify_options) {
      Options options(verify_options);

      if (options.heart_beat_time < std::chrono::milliseconds(100))
        options.heart_beat_time = std::chrono::milliseconds(100);

      return options;
    }
  };

  explicit AsioWebSocketClient(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioWebSocketClient() = default;

  AsioWebSocketClient(const AsioWebSocketClient&) = delete;
  AsioWebSocketClient& operator=(const AsioWebSocketClient&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    logger_ptr_ = logger_ptr;
  }

  template <typename... Args>
    requires std::constructible_from<MsgHandle, Args...>
  void RegisterMsgHandle(Args&&... args) {
    AIMRT_CHECK_ERROR_THROW(
        state_.load() == State::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    msg_handle_ptr_ = std::make_shared<MsgHandle>(std::forward<Args>(args)...);
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
        AIMRT_WARN("WebSocket cli is closed, will not send current msg.");
        return;
      }

      if (!session_ptr_ || !session_ptr_->IsRunning()) {
        session_ptr_ = std::make_shared<Session>(
            io_ptr_, logger_ptr_, msg_handle_ptr_);
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
        : host(options.host),
          service(options.service),
          path(options.path),
          heart_beat_time(options.heart_beat_time),
          max_recv_size(options.max_recv_size),
          binary_mode(options.binary_mode) {}

    std::string host;
    std::string service;
    std::string path;
    std::chrono::nanoseconds heart_beat_time;
    uint32_t max_recv_size;
    bool binary_mode;
  };

  class Session : public std::enable_shared_from_this<Session> {
   public:
    Session(const std::shared_ptr<IOCtx>& io_ptr,
            const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr,
            const std::shared_ptr<MsgHandle>& msg_handle_ptr)
        : io_ptr_(io_ptr),
          session_socket_strand_(boost::asio::make_strand(*io_ptr)),
          stream_(session_socket_strand_),
          send_sig_timer_(session_socket_strand_),
          logger_ptr_(logger_ptr),
          msg_handle_ptr_(msg_handle_ptr) {}

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

      auto self = shared_from_this();

      boost::asio::co_spawn(
          session_socket_strand_,
          [this, self]() -> Awaitable<void> {
            try {
              namespace asio = boost::asio;
              namespace http = boost::beast::http;
              namespace websocket = boost::beast::websocket;

              AIMRT_TRACE("WebSocket cli session create a new connect to {}:{}",
                          session_options_ptr_->host,
                          session_options_ptr_->service);

              // resolve
              asio::ip::tcp::resolver resolver(session_socket_strand_);
              auto const dst = co_await resolver.async_resolve(
                  session_options_ptr_->host,
                  session_options_ptr_->service,
                  asio::use_awaitable);

              stream_.next_layer().expires_after(std::chrono::seconds(5));
              co_await stream_.next_layer().async_connect(dst, asio::use_awaitable);

              remote_addr_ = aimrt::common::util::SSToString(stream_.next_layer().socket().remote_endpoint());
              AIMRT_TRACE("WebSocket cli session async connect, remote addr {}.", RemoteAddr());

              stream_.next_layer().expires_never();

              stream_.set_option(websocket::stream_base::timeout::suggested(
                  boost::beast::role_type::client));

              stream_.set_option(websocket::stream_base::decorator(
                  [](websocket::request_type& req) {
                    req.set(http::field::user_agent,
                            std::string(BOOST_BEAST_VERSION_STRING) + " websocket-client-coro");
                  }));

              co_await stream_.async_handshake(remote_addr_, session_options_ptr_->path, asio::use_awaitable);
              AIMRT_TRACE("WebSocket cli session async handshake, path {}, remote addr {}.",
                          session_options_ptr_->path, RemoteAddr());

              stream_.binary(session_options_ptr_->binary_mode);

              // Sender co
              boost::asio::co_spawn(
                  session_socket_strand_,
                  [this, self]() -> Awaitable<void> {
                    try {
                      while (state_.load() == SessionState::kStart) {
                        while (!data_list_.empty()) {
                          auto data_itr = data_list_.begin();

                          size_t write_data_size = co_await stream_.async_write(
                              (*data_itr)->data(), boost::asio::use_awaitable);
                          AIMRT_TRACE(
                              "WebSocket cli session async write {} bytes to {}.",
                              write_data_size, RemoteAddr());
                          data_list_.erase(data_itr);
                        }

                        bool heartbeat_flag = false;
                        try {
                          send_sig_timer_.expires_after(session_options_ptr_->heart_beat_time);
                          co_await send_sig_timer_.async_wait(boost::asio::use_awaitable);
                          heartbeat_flag = true;
                        } catch (const std::exception& e) {
                          AIMRT_TRACE(
                              "WebSocket cli session timer canceled, remote addr {}, exception info: {}",
                              RemoteAddr(), e.what());
                        }

                        if (heartbeat_flag) {
                          // Heartbeat packet is only used to keep alive and does not transmit business/management information
                          static const websocket::ping_data kEmptyPingData;

                          co_await stream_.async_ping(kEmptyPingData, boost::asio::use_awaitable);
                          AIMRT_TRACE(
                              "WebSocket cli session async ping to {} for heartbeat.",
                              RemoteAddr());
                        }
                      }
                    } catch (const std::exception& e) {
                      AIMRT_TRACE(
                          "WebSocket cli session send co get exception and exit, remote addr {}, exception info: {}",
                          RemoteAddr(), e.what());
                    }

                    Shutdown();

                    co_return;
                  },
                  boost::asio::detached);

              // Receiver co
              boost::asio::co_spawn(
                  session_socket_strand_,
                  [this, self]() -> Awaitable<void> {
                    try {
                      while (state_.load() == SessionState::kStart) {
                        auto msg_buf = std::make_shared<Streambuf>();

                        size_t read_data_size = co_await stream_.async_read(
                            *msg_buf, boost::asio::use_awaitable);
                        AIMRT_TRACE(
                            "WebSocket cli session async read {} bytes from {} for head.",
                            read_data_size, RemoteAddr());

                        if (msg_handle_ptr_) {
                          boost::asio::post(
                              *io_ptr_,
                              [this, msg_buf]() { (*msg_handle_ptr_)(msg_buf); });
                        }
                      }
                    } catch (const std::exception& e) {
                      AIMRT_TRACE(
                          "WebSocket cli session recv co get exception and exit, remote addr {}, exception info: {}",
                          RemoteAddr(), e.what());
                    }

                    Shutdown();

                    co_return;
                  },
                  boost::asio::detached);

            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "WebSocket cli session start co get exception and exit, remote addr {}, exception info: {}",
                  RemoteAddr(), e.what());
              Shutdown();
            }

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
                send_sig_timer_.cancel();
                ++stop_step;
              case 2:
                stream_.next_layer().socket().shutdown(Tcp::socket::shutdown_both);
                ++stop_step;
              case 3:
                stream_.next_layer().socket().cancel();
                ++stop_step;
              case 4:
                stream_.next_layer().socket().close();
                ++stop_step;
              case 5:
                stream_.next_layer().socket().release();
                ++stop_step;
              case 6:
                stream_.next_layer().cancel();
                ++stop_step;
              case 7:
                stream_.next_layer().close();
                ++stop_step;
              case 8:
                stream_.next_layer().release_socket();
                ++stop_step;
              case 9:
                stream_.close(boost::beast::websocket::close_code::normal);
                ++stop_step;
              default:
                stop_step = 0;
                break;
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE(
                "WebSocket cli session stop get exception at step {}, exception info: {}",
                stop_step, e.what());
            ++stop_step;
          }
        }
      });
    }

    void SendMsg(const std::shared_ptr<Streambuf>& msg_buf_ptr) {
      // TODO: There are two situations. When the connection has not been established, it is placed in the cache first, and after the connection is established, it is sent directly.
      auto self = shared_from_this();
      boost::asio::dispatch(
          session_socket_strand_,
          [this, self, msg_buf_ptr]() {
            if (state_.load() != SessionState::kStart) [[unlikely]] {
              AIMRT_WARN("WebSocket cli session is closed, will not send current msg.");
              return;
            }

            data_list_.emplace_back(msg_buf_ptr);
            send_sig_timer_.cancel();
          });
    }

    const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

    std::string_view RemoteAddr() const { return remote_addr_; }

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
    Strand session_socket_strand_;
    WsStream stream_;
    Timer send_sig_timer_;

    // Log handle
    std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

    // Msg processing handle
    std::shared_ptr<MsgHandle> msg_handle_ptr_;

    // Options
    std::shared_ptr<const SessionOptions> session_options_ptr_;

    // State
    std::atomic<SessionState> state_ = SessionState::kPreInit;

    // misc
    std::string remote_addr_;
    std::list<std::shared_ptr<Streambuf>> data_list_;
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

  // Msg processing handle
  std::shared_ptr<MsgHandle> msg_handle_ptr_;

  // Options
  Options options_;

  // State
  std::atomic<State> state_ = State::kPreInit;

  // Session management
  std::shared_ptr<const SessionOptions> session_options_ptr_;
  std::shared_ptr<Session> session_ptr_;
};

class AsioWebSocketClientPool
    : public std::enable_shared_from_this<AsioWebSocketClientPool> {
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

  explicit AsioWebSocketClientPool(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioWebSocketClientPool() = default;

  AsioWebSocketClientPool(const AsioWebSocketClientPool&) = delete;
  AsioWebSocketClientPool& operator=(const AsioWebSocketClientPool&) = delete;

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

  Awaitable<std::shared_ptr<AsioWebSocketClient>> GetClient(
      const AsioWebSocketClient::Options& client_options) {
    return boost::asio::co_spawn(
        mgr_strand_,
        [this, &client_options]() -> Awaitable<std::shared_ptr<AsioWebSocketClient>> {
          if (state_.load() != State::kStart) [[unlikely]] {
            AIMRT_WARN("WebSocket cli pool is closed, will not return cli instance.");
            co_return std::shared_ptr<AsioWebSocketClient>();
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

            AIMRT_CHECK_WARN_THROW(client_map_.size() < options_.max_client_num,
                                   "WebSocket client num reach the upper limit.");
          }

          auto client_ptr = std::make_shared<AsioWebSocketClient>(io_ptr_);
          client_ptr->SetLogger(logger_ptr_);
          client_ptr->Initialize(client_options);
          client_ptr->Start();

          client_map_.emplace(std::move(client_key), client_ptr);
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
  std::unordered_map<std::string, std::shared_ptr<AsioWebSocketClient>> client_map_;
};

}  // namespace aimrt::common::net
