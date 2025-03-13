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
#include <utility>

#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::common::net {

class AsioWebSocketServer : public std::enable_shared_from_this<AsioWebSocketServer> {
 public:
  using IOCtx = boost::asio::io_context;
  using Tcp = boost::asio::ip::tcp;
  using WsStream = boost::beast::websocket::stream<boost::beast::tcp_stream>;
  using Strand = boost::asio::strand<IOCtx::executor_type>;
  using Timer = boost::asio::steady_timer;
  using Streambuf = boost::asio::streambuf;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  using MsgHandle = std::function<void(const Tcp::endpoint&, const std::shared_ptr<Streambuf>&)>;

  struct Options {
    /// Listening address
    Tcp::endpoint ep = Tcp::endpoint{boost::asio::ip::address_v4(), 50180};

    /// Maximum number of connections
    size_t max_session_num = 1000000;

    /// Managing coroutine timer intervals
    std::chrono::nanoseconds mgr_timer_dt = std::chrono::seconds(10);

    /// Maximum time without data
    std::chrono::nanoseconds max_no_data_duration = std::chrono::seconds(300);

    /// Maximum package size
    uint32_t max_recv_size = 1024 * 1024 * 16;

    // Use binary mode or text mode
    bool binary_mode = true;

    static Options Verify(const Options& verify_options) {
      Options options(verify_options);

      if (options.max_session_num < 1) options.max_session_num = 1;

      if (options.max_session_num > Tcp::acceptor::max_listen_connections)
        options.max_session_num = Tcp::acceptor::max_listen_connections;

      if (options.mgr_timer_dt < std::chrono::milliseconds(100))
        options.mgr_timer_dt = std::chrono::milliseconds(100);

      if (options.max_no_data_duration < std::chrono::seconds(10))
        options.max_no_data_duration = std::chrono::seconds(10);

      return options;
    }
  };

  explicit AsioWebSocketServer(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        acceptor_(mgr_strand_),
        acceptor_timer_(mgr_strand_),
        mgr_timer_(mgr_strand_),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioWebSocketServer() = default;

  AsioWebSocketServer(const AsioWebSocketServer&) = delete;
  AsioWebSocketServer& operator=(const AsioWebSocketServer&) = delete;

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

    AIMRT_CHECK_ERROR_THROW(CheckListenAddr(options_.ep),
                            "{} is already in use.", aimrt::common::util::SSToString(options_.ep));
  }

  void Start() {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, State::kStart) == State::kInit,
        "Method can only be called when state is 'Init'.");

    auto self = shared_from_this();
    boost::asio::co_spawn(
        mgr_strand_,
        [this, self]() -> Awaitable<void> {
          acceptor_.open(options_.ep.protocol());
          acceptor_.set_option(Tcp::acceptor::reuse_address(true));
          acceptor_.bind(options_.ep);
          acceptor_.listen();

          while (state_.load() == State::kStart) {
            try {
              // If the number of links reaches the upper limit, wait for a while and try again
              if (session_ptr_map_.size() >= options_.max_session_num) {
                acceptor_timer_.expires_after(options_.mgr_timer_dt);
                co_await acceptor_timer_.async_wait(boost::asio::use_awaitable);
                continue;
              }

              auto session_ptr = std::make_shared<Session>(
                  io_ptr_, logger_ptr_, msg_handle_ptr_);
              session_ptr->Initialize(session_options_ptr_);

              co_await acceptor_.async_accept(session_ptr->Socket(),
                                              boost::asio::use_awaitable);
              session_ptr->Start();

              session_ptr_map_.emplace(session_ptr->Socket().remote_endpoint(), session_ptr);

            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "WebSocket svr accept connection get exception and exit, exception info: {}",
                  e.what());
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

              for (auto itr = session_ptr_map_.begin(); itr != session_ptr_map_.end();) {
                if (itr->second->IsRunning())
                  ++itr;
                else
                  session_ptr_map_.erase(itr++);
              }
            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "WebSocket svr timer get exception and exit, exception info: {}",
                  e.what());
            }
          }

          Shutdown();

          co_return;
        },
        boost::asio::detached);
  }

  void Shutdown() {
    if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
      return;

    auto self = shared_from_this();
    boost::asio::dispatch(mgr_strand_, [this, self]() {
      uint32_t stop_step = 1;
      while (stop_step) {
        try {
          switch (stop_step) {
            case 1:
              acceptor_timer_.cancel();
              ++stop_step;
            case 2:
              mgr_timer_.cancel();
              ++stop_step;
            case 3:
              acceptor_.cancel();
              ++stop_step;
            case 4:
              acceptor_.close();
              ++stop_step;
            case 5:
              acceptor_.release();
              ++stop_step;
            default:
              stop_step = 0;
              break;
          }
        } catch (const std::exception& e) {
          AIMRT_WARN(
              "WebSocket svr stop get exception at step {}, exception info: {}",
              stop_step, e.what());
          ++stop_step;
        }
      }

      for (auto& session_ptr : session_ptr_map_) session_ptr.second->Shutdown();

      session_ptr_map_.clear();
    });
  }

  void SendMsg(const Tcp::endpoint& ep, const std::shared_ptr<Streambuf>& msg_buf_ptr) {
    auto self = shared_from_this();
    boost::asio::dispatch(mgr_strand_, [this, self, ep, msg_buf_ptr]() {
      if (state_.load() != State::kStart) [[unlikely]] {
        AIMRT_WARN("WebSocket svr is closed, will not send current msg.");
        return;
      }

      auto finditr = session_ptr_map_.find(ep);
      if (finditr == session_ptr_map_.end()) {
        AIMRT_WARN("WebSocket svr can not find endpoint {} in session map",
                   aimrt::common::util::SSToString(ep));
        return;
      }

      auto session_ptr = finditr->second;
      if (session_ptr && session_ptr->IsRunning()) {
        session_ptr->SendMsg(msg_buf_ptr);
      }
    });
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  bool IsRunning() const { return state_.load() == State::kStart; }

 private:
  static bool CheckListenAddr(const Tcp::endpoint& ep) {
    try {
      IOCtx io;
      Tcp::acceptor acceptor(io, ep);
      return true;
    } catch (...) {
      return false;
    }
  }

  struct SessionOptions {
    explicit SessionOptions(const Options& options)
        : max_no_data_duration(options.max_no_data_duration),
          max_recv_size(options.max_recv_size),
          binary_mode(options.binary_mode) {}

    std::chrono::nanoseconds max_no_data_duration;
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
          session_mgr_strand_(boost::asio::make_strand(*io_ptr)),
          timer_(session_socket_strand_),
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

      remote_addr_ = aimrt::common::util::SSToString(stream_.next_layer().socket().remote_endpoint());
      AIMRT_TRACE("WebSocket svr accept a new connect from {}.", RemoteAddr());

      auto self = shared_from_this();

      // Establishing a connection
      boost::asio::co_spawn(
          session_socket_strand_,
          [this, self]() -> Awaitable<void> {
            try {
              namespace http = boost::beast::http;
              namespace websocket = boost::beast::websocket;

              stream_.set_option(websocket::stream_base::timeout::suggested(
                  boost::beast::role_type::server));

              stream_.set_option(websocket::stream_base::decorator(
                  [](websocket::response_type& res) {
                    res.set(http::field::server,
                            std::string(BOOST_BEAST_VERSION_STRING) + " websocket-server-coro");
                  }));

              co_await stream_.async_accept(boost::asio::use_awaitable);

              stream_.binary(session_options_ptr_->binary_mode);

              // Sending coroutines
              boost::asio::co_spawn(
                  session_socket_strand_,
                  [this, self]() -> Awaitable<void> {
                    try {
                      while (state_.load() == SessionState::kStart) {
                        while (!data_list_.empty()) {
                          auto data_itr = data_list_.begin();

                          tick_has_data_ = true;
                          size_t write_data_size = co_await stream_.async_write(
                              (*data_itr)->data(), boost::asio::use_awaitable);
                          AIMRT_TRACE("WebSocket svr session async write {} bytes to {}.",
                                      write_data_size, RemoteAddr());
                          data_list_.erase(data_itr);
                        }

                        try {
                          send_sig_timer_.expires_after(std::chrono::seconds(3600));
                          co_await send_sig_timer_.async_wait(boost::asio::use_awaitable);
                        } catch (const std::exception& e) {
                          AIMRT_TRACE(
                              "WebSocket svr session timer canceled, remote addr {}, exception info: {}",
                              RemoteAddr(), e.what());
                        }
                      }
                    } catch (const std::exception& e) {
                      AIMRT_WARN(
                          "WebSocket svr session send co get exception and exit, remote addr {}, exception info: {}",
                          RemoteAddr(), e.what());
                    }

                    Shutdown();

                    co_return;
                  },
                  boost::asio::detached);

              // Receiving coroutine
              boost::asio::co_spawn(
                  session_socket_strand_,
                  [this, self]() -> Awaitable<void> {
                    try {
                      while (state_.load() == SessionState::kStart) {
                        auto msg_buf = std::make_shared<Streambuf>();

                        size_t read_data_size = co_await stream_.async_read(
                            *msg_buf, boost::asio::use_awaitable);
                        AIMRT_TRACE("WebSocket svr session async read {} bytes from {}.",
                                    read_data_size, RemoteAddr());
                        tick_has_data_ = true;

                        boost::asio::post(
                            *io_ptr_,
                            [this, msg_buf]() { (*msg_handle_ptr_)(Socket().remote_endpoint(), msg_buf); });
                      }
                    } catch (const std::exception& e) {
                      AIMRT_WARN(
                          "WebSocket svr session recv co get exception and exit, remote addr {}, exception info: {}",
                          RemoteAddr(), e.what());
                    }

                    Shutdown();

                    co_return;
                  },
                  boost::asio::detached);

            } catch (const std::exception& e) {
              AIMRT_WARN(
                  "WebSocket svr session get exception and exit, remote addr {}, exception info: {}",
                  RemoteAddr(), e.what());
              Shutdown();
            }

            co_return;
          },
          boost::asio::detached);

      // Timer coroutine
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
                      "WebSocket svr session exit due to timeout({}ms), remote addr {}.",
                      std::chrono::duration_cast<std::chrono::milliseconds>(
                          session_options_ptr_->max_no_data_duration)
                          .count(),
                      RemoteAddr());
                  break;
                }
              }
            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "WebSocket svr session timer get exception and exit, remote addr {}, exception info: {}",
                  RemoteAddr(), e.what());
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
                "WebSocket svr session stop get exception at step {}, remote addr {}, exception info: {}",
                RemoteAddr(), stop_step, e.what());
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
            AIMRT_WARN(
                "WebSocket svr session mgr stop get exception at step {}, remote addr {}, exception info: {}",
                stop_step, RemoteAddr(), e.what());
            ++stop_step;
          }
        }
      });
    }

    void SendMsg(const std::shared_ptr<Streambuf>& msg_buf_ptr) {
      auto self = shared_from_this();
      boost::asio::dispatch(
          session_socket_strand_,
          [this, self, msg_buf_ptr]() {
            if (state_.load() != SessionState::kStart) [[unlikely]] {
              AIMRT_WARN("WebSocket svr session is closed, will not send current msg.");
              return;
            }

            data_list_.emplace_back(msg_buf_ptr);
            send_sig_timer_.cancel();
          });
    }

    const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

    Tcp::socket& Socket() { return stream_.next_layer().socket(); }

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
    Strand session_mgr_strand_;
    Timer timer_;

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
    std::atomic_bool tick_has_data_ = false;
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
  Strand mgr_strand_;       // Session pool operation strand
  Tcp::acceptor acceptor_;  // Listener
  Timer acceptor_timer_;    // The sleep timer when the connection is full
  Timer mgr_timer_;         // Timer to manage session pool

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
  std::unordered_map<Tcp::endpoint, std::shared_ptr<Session>> session_ptr_map_;
};

}  // namespace aimrt::common::net
