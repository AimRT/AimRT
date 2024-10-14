// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <chrono>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include <boost/asio.hpp>

#include "util/buffer_util.h"
#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::common::net {

class AsioTcpServer : public std::enable_shared_from_this<AsioTcpServer> {
 public:
  using IOCtx = boost::asio::io_context;
  using Tcp = boost::asio::ip::tcp;
  using Strand = boost::asio::strand<IOCtx::executor_type>;
  using Timer = boost::asio::steady_timer;
  using Streambuf = boost::asio::streambuf;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  using MsgHandle =
      std::function<void(const Tcp::endpoint&, const std::shared_ptr<Streambuf>&)>;

  struct Options {
    /// 监听的地址
    Tcp::endpoint ep = Tcp::endpoint{boost::asio::ip::address_v4(), 57634};

    /// 最大连接数
    size_t max_session_num = 1000000;

    /// 管理协程定时器间隔
    std::chrono::nanoseconds mgr_timer_dt = std::chrono::seconds(10);

    /// 最长无数据时间
    std::chrono::nanoseconds max_no_data_duration = std::chrono::seconds(300);

    /// 包最大尺寸
    uint32_t max_recv_size = 1024 * 1024 * 16;

    /// 校验配置
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

  explicit AsioTcpServer(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        acceptor_(mgr_strand_),
        acceptor_timer_(mgr_strand_),
        mgr_timer_(mgr_strand_),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioTcpServer() = default;

  AsioTcpServer(const AsioTcpServer&) = delete;
  AsioTcpServer& operator=(const AsioTcpServer&) = delete;

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
        msg_handle_ptr_,
        "Msg handle is not set before initialize.");

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
              // 如果链接数达到上限，则等待一段时间再试
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
              AIMRT_TRACE(
                  "Tcp svr accept a new connection, remote addr is {}",
                  aimrt::common::util::SSToString(session_ptr->Socket().remote_endpoint()));
              session_ptr->Start();

              session_ptr_map_.emplace(session_ptr->Socket().remote_endpoint(), session_ptr);

            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "Tcp svr accept connection get exception and exit, exception info: {}",
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
                  "Tcp svr timer get exception and exit, exception info: {}",
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
              "Tcp svr stop get exception at step {}, exception info: {}",
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
        AIMRT_WARN("Tcp svr is closed, will not send current msg.");
        return;
      }

      auto finditr = session_ptr_map_.find(ep);
      if (finditr == session_ptr_map_.end()) {
        AIMRT_WARN("Tcp svr can not find endpoint {} in session map",
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
  // 包头结构：| 2byte magicnum | 4byte msglen |
  static constexpr size_t kHeadSize = 6;
  static constexpr char kHeadByte1 = 'Y';
  static constexpr char kHeadByte2 = 'T';

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
          max_recv_size(options.max_recv_size) {}

    std::chrono::nanoseconds max_no_data_duration;
    uint32_t max_recv_size;
  };

  class Session : public std::enable_shared_from_this<Session> {
   public:
    Session(const std::shared_ptr<IOCtx>& io_ptr,
            const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr,
            const std::shared_ptr<MsgHandle>& msg_handle_ptr)
        : io_ptr_(io_ptr),
          session_socket_strand_(boost::asio::make_strand(*io_ptr)),
          sock_(session_socket_strand_),
          send_sig_timer_(session_socket_strand_),
          session_mgr_strand_(boost::asio::make_strand(*io_ptr)),
          timer_(session_mgr_strand_),
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

      remote_addr_ = aimrt::common::util::SSToString(sock_.remote_endpoint());

      auto self = shared_from_this();

      // 发送协程
      boost::asio::co_spawn(
          session_socket_strand_,
          [this, self]() -> Awaitable<void> {
            try {
              while (state_.load() == SessionState::kStart) {
                while (!data_list_.empty()) {
                  std::list<std::shared_ptr<Streambuf>> tmp_data_list;
                  tmp_data_list.swap(data_list_);

                  std::vector<char> head_buf(tmp_data_list.size() * kHeadSize);

                  std::vector<boost::asio::const_buffer> data_buf_vec;
                  data_buf_vec.reserve(tmp_data_list.size() * 2);
                  size_t ct = 0;
                  for (auto& itr : tmp_data_list) {
                    head_buf[ct * kHeadSize] = kHeadByte1;
                    head_buf[ct * kHeadSize + 1] = kHeadByte2;
                    aimrt::common::util::SetBufFromUint32(&head_buf[ct * kHeadSize + 2],
                                                          static_cast<uint32_t>(itr->size()));
                    data_buf_vec.emplace_back(
                        boost::asio::const_buffer(&head_buf[ct * kHeadSize], kHeadSize));
                    ++ct;

                    data_buf_vec.emplace_back(itr->data());
                  }

                  tick_has_data_ = true;
                  size_t write_data_size = co_await boost::asio::async_write(
                      sock_, data_buf_vec, boost::asio::use_awaitable);
                  AIMRT_TRACE("Tcp svr session async write {} bytes to {}.",
                              write_data_size, RemoteAddr());
                }

                try {
                  send_sig_timer_.expires_after(std::chrono::seconds(3600));
                  co_await send_sig_timer_.async_wait(boost::asio::use_awaitable);
                } catch (const std::exception& e) {
                  AIMRT_TRACE(
                      "Tcp svr session timer canceled, remote addr {}, exception info: {}",
                      RemoteAddr(), e.what());
                }
              }
            } catch (const std::exception& e) {
              AIMRT_WARN(
                  "Tcp svr session send co get exception and exit, remote addr {}, exception info: {}",
                  RemoteAddr(), e.what());
            }

            Shutdown();

            co_return;
          },
          boost::asio::detached);

      // 接收协程
      boost::asio::co_spawn(
          session_socket_strand_,
          [this, self]() -> Awaitable<void> {
            try {
              std::vector<char> head_buf(kHeadSize);
              boost::asio::mutable_buffer asio_head_buf(head_buf.data(), kHeadSize);

              while (state_.load() == SessionState::kStart) {
                size_t read_data_size = co_await boost::asio::async_read(
                    sock_, asio_head_buf,
                    boost::asio::transfer_exactly(kHeadSize),
                    boost::asio::use_awaitable);
                AIMRT_TRACE(
                    "Tcp svr session async read {} bytes from {} for head.",
                    read_data_size, RemoteAddr());
                tick_has_data_ = true;

                AIMRT_CHECK_ERROR_THROW(
                    read_data_size == kHeadSize && head_buf[0] == kHeadByte1 && head_buf[1] == kHeadByte2,
                    "Get an invalid head, remote addr {}, read_data_size: {}, head_buf[0]: {}, head_buf[1]: {}.",
                    RemoteAddr(), read_data_size,
                    static_cast<uint8_t>(head_buf[0]),
                    static_cast<uint8_t>(head_buf[1]));

                uint32_t msg_len = aimrt::common::util::GetUint32FromBuf(&head_buf[2]);

                AIMRT_CHECK_ERROR_THROW(
                    msg_len <= session_options_ptr_->max_recv_size,
                    "Msg too large, remote addr {}, size: {}.", RemoteAddr(), msg_len);

                auto msg_buf = std::make_shared<Streambuf>();

                read_data_size = co_await boost::asio::async_read(
                    sock_, msg_buf->prepare(msg_len),
                    boost::asio::transfer_exactly(msg_len),
                    boost::asio::use_awaitable);
                AIMRT_TRACE("Tcp svr session async read {} bytes from {}.",
                            read_data_size, RemoteAddr());
                tick_has_data_ = true;

                msg_buf->commit(msg_len);
                boost::asio::post(
                    *io_ptr_,
                    [this, msg_buf]() { (*msg_handle_ptr_)(Socket().remote_endpoint(), msg_buf); });
              }
            } catch (const std::exception& e) {
              AIMRT_WARN(
                  "Tcp svr session recv co get exception and exit, remote addr {}, exception info: {}",
                  RemoteAddr(), e.what());
            }

            Shutdown();

            co_return;
          },
          boost::asio::detached);

      // 定时器协程
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
                      "Tcp svr session exit due to timeout({}ms), remote addr {}.",
                      std::chrono::duration_cast<std::chrono::milliseconds>(session_options_ptr_->max_no_data_duration).count(),
                      RemoteAddr());
                  break;
                }
              }
            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "Tcp svr session timer get exception and exit, remote addr {}, exception info: {}",
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
                sock_.shutdown(Tcp::socket::shutdown_both);
                ++stop_step;
              case 3:
                sock_.cancel();
                ++stop_step;
              case 4:
                sock_.close();
                ++stop_step;
              case 5:
                sock_.release();
                ++stop_step;
              default:
                stop_step = 0;
                break;
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE(
                "Tcp svr session stop get exception at step {}, remote addr {}, exception info: {}",
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
                "Tcp svr session mgr stop get exception at step {}, remote addr {}, exception info: {}",
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
              AIMRT_WARN("Tcp svr session is closed, will not send current msg.");
              return;
            }

            data_list_.emplace_back(msg_buf_ptr);
            send_sig_timer_.cancel();
          });
    }

    const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

    Tcp::socket& Socket() { return sock_; }

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
    Tcp::socket sock_;
    Timer send_sig_timer_;
    Strand session_mgr_strand_;
    Timer timer_;

    // 日志打印句柄
    std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

    // msg处理句柄
    std::shared_ptr<MsgHandle> msg_handle_ptr_;

    // 配置
    std::shared_ptr<const SessionOptions> session_options_ptr_;

    // 状态
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
  Strand mgr_strand_;       // session池操作strand
  Tcp::acceptor acceptor_;  // 监听器
  Timer acceptor_timer_;    // 连接满时监听器的sleep定时器
  Timer mgr_timer_;         // 管理session池的定时器

  // 日志打印句柄
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  // msg处理句柄
  std::shared_ptr<MsgHandle> msg_handle_ptr_;

  // 配置
  Options options_;

  // 状态
  std::atomic<State> state_ = State::kPreInit;

  // session管理
  std::shared_ptr<const SessionOptions> session_options_ptr_;
  std::unordered_map<Tcp::endpoint, std::shared_ptr<Session>> session_ptr_map_;
};

}  // namespace aimrt::common::net
