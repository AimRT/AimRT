// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <chrono>
#include <list>
#include <memory>
#include <unordered_map>

#include <boost/asio.hpp>

#include "util/buffer_util.h"
#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::common::net {

class AsioTcpClient : public std::enable_shared_from_this<AsioTcpClient> {
 public:
  using IOCtx = boost::asio::io_context;
  using Tcp = boost::asio::ip::tcp;
  using Strand = boost::asio::strand<IOCtx::executor_type>;
  using Timer = boost::asio::steady_timer;
  using Streambuf = boost::asio::streambuf;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  using MsgHandle = std::function<void(const std::shared_ptr<Streambuf>&)>;

  struct Options {
    /// 服务端地址
    Tcp::endpoint svr_ep;

    /// 定时器间隔
    std::chrono::nanoseconds heart_beat_time = std::chrono::seconds(60);

    /// 包最大尺寸，最大10m
    uint32_t max_recv_size = 1024 * 1024 * 10;

    /// 校验配置
    static Options Verify(const Options& verify_options) {
      Options options(verify_options);

      if (options.heart_beat_time < std::chrono::milliseconds(100))
        options.heart_beat_time = std::chrono::milliseconds(100);

      return options;
    }
  };

  explicit AsioTcpClient(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioTcpClient() = default;

  AsioTcpClient(const AsioTcpClient&) = delete;
  AsioTcpClient& operator=(const AsioTcpClient&) = delete;

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
        AIMRT_WARN("Tcp cli is closed, will not send current msg.");
        return;
      }

      if (!session_ptr_ || !session_ptr_->IsRunning()) {
        session_ptr_ = std::make_shared<Session>(io_ptr_, logger_ptr_, msg_handle_ptr_);
        session_ptr_->Initialize(session_options_ptr_);
        session_ptr_->Start();
      }

      session_ptr_->SendMsg(msg_buf_ptr);
    });
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  bool IsRunning() const { return state_.load() == State::kStart; }

 private:
  // 包头结构：| 2byte magicnum | 4byte msglen |
  static constexpr size_t kHeadSize = 6;
  static constexpr char kHeadByte1 = 'Y';
  static constexpr char kHeadByte2 = 'T';

  struct SessionOptions {
    explicit SessionOptions(const Options& options)
        : svr_ep(options.svr_ep),
          heart_beat_time(options.heart_beat_time),
          max_recv_size(options.max_recv_size) {}

    Tcp::endpoint svr_ep;
    std::chrono::nanoseconds heart_beat_time;
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

      // 启动协程
      boost::asio::co_spawn(
          session_socket_strand_,
          [this, self]() -> Awaitable<void> {
            try {
              remote_addr_ = aimrt::common::util::SSToString(session_options_ptr_->svr_ep);

              AIMRT_TRACE("Tcp cli session create a new connect to {}.", RemoteAddr());
              co_await sock_.async_connect(session_options_ptr_->svr_ep,
                                           boost::asio::use_awaitable);

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
                            aimrt::common::util::SetBufFromUint32(
                                &head_buf[ct * kHeadSize + 2],
                                static_cast<uint32_t>(itr->size()));
                            data_buf_vec.emplace_back(
                                boost::asio::const_buffer(&head_buf[ct * kHeadSize], kHeadSize));
                            ++ct;

                            data_buf_vec.emplace_back(itr->data());
                          }

                          size_t write_data_size = co_await boost::asio::async_write(
                              sock_, data_buf_vec, boost::asio::use_awaitable);
                          AIMRT_TRACE(
                              "Tcp cli session async write {} bytes to {}.",
                              write_data_size, RemoteAddr());
                        }

                        bool heartbeat_flag = false;
                        try {
                          send_sig_timer_.expires_after(session_options_ptr_->heart_beat_time);
                          co_await send_sig_timer_.async_wait(boost::asio::use_awaitable);
                          heartbeat_flag = true;
                        } catch (const std::exception& e) {
                          AIMRT_TRACE(
                              "Tcp cli session timer canceled, remote addr {}, exception info: {}",
                              RemoteAddr(), e.what());
                        }

                        if (heartbeat_flag) {
                          // 心跳包仅用来保活，不传输业务/管理信息
                          static constexpr char kHeartbeatPkg[kHeadSize] = {
                              kHeadByte1, kHeadByte2, 0, 0, 0, 0};
                          static const boost::asio::const_buffer kHeartbeatBuf(
                              kHeartbeatPkg, kHeadSize);

                          size_t write_data_size =
                              co_await boost::asio::async_write(
                                  sock_, kHeartbeatBuf,
                                  boost::asio::use_awaitable);
                          AIMRT_TRACE(
                              "Tcp cli session async write {} bytes to {} for heartbeat.",
                              write_data_size, RemoteAddr());
                        }
                      }
                    } catch (const std::exception& e) {
                      AIMRT_TRACE(
                          "Tcp cli session send co get exception and exit, remote addr {}, exception info: {}",
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
                            "Tcp cli session async read {} bytes from {} for head.",
                            read_data_size, RemoteAddr());

                        AIMRT_CHECK_WARN_THROW(
                            read_data_size == kHeadSize && head_buf[0] == kHeadByte1 && head_buf[1] == kHeadByte2,
                            "Get an invalid head, remote addr {}, read_data_size: {}, head_buf[0]: {}, head_buf[1]: {}.",
                            RemoteAddr(), read_data_size,
                            static_cast<uint8_t>(head_buf[0]),
                            static_cast<uint8_t>(head_buf[1]));

                        uint32_t msg_len = aimrt::common::util::GetUint32FromBuf(&head_buf[2]);

                        AIMRT_CHECK_WARN_THROW(
                            msg_len <= session_options_ptr_->max_recv_size,
                            "Msg too large, remote addr {}, size: {}.",
                            RemoteAddr(), msg_len);

                        auto msg_buf = std::make_shared<Streambuf>();

                        read_data_size = co_await boost::asio::async_read(
                            sock_, msg_buf->prepare(msg_len),
                            boost::asio::transfer_exactly(msg_len),
                            boost::asio::use_awaitable);
                        AIMRT_TRACE(
                            "Tcp cli session async read {} bytes from {}.",
                            read_data_size, RemoteAddr());

                        msg_buf->commit(msg_len);

                        if (msg_handle_ptr_) {
                          boost::asio::post(
                              *io_ptr_,
                              [this, msg_buf]() { (*msg_handle_ptr_)(msg_buf); });
                        }
                      }
                    } catch (const std::exception& e) {
                      AIMRT_TRACE(
                          "Tcp cli session recv co get exception and exit, remote addr {}, exception info: {}",
                          RemoteAddr(), e.what());
                    }

                    Shutdown();

                    co_return;
                  },
                  boost::asio::detached);

            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "Tcp cli session start co get exception and exit, remote addr {}, exception info: {}",
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
                "Tcp cli session stop get exception at step {}, remote addr {}, exception info: {}",
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
              AIMRT_WARN("Tcp cli session is closed, will not send current msg.");
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
    Tcp::socket sock_;
    Timer send_sig_timer_;

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
  std::shared_ptr<Session> session_ptr_;
};

class AsioTcpClientPool
    : public std::enable_shared_from_this<AsioTcpClientPool> {
 public:
  using IOCtx = boost::asio::io_context;
  using Strand = boost::asio::strand<IOCtx::executor_type>;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  struct Options {
    /// 最大client数
    size_t max_client_num = 1000;

    /// 校验配置
    static Options Verify(const Options& verify_options) {
      Options options(verify_options);

      if (options.max_client_num < 10) options.max_client_num = 10;

      return options;
    }
  };

  explicit AsioTcpClientPool(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioTcpClientPool() = default;

  AsioTcpClientPool(const AsioTcpClientPool&) = delete;
  AsioTcpClientPool& operator=(const AsioTcpClientPool&) = delete;

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

  Awaitable<std::shared_ptr<AsioTcpClient>> GetClient(
      const AsioTcpClient::Options& client_options) {
    return boost::asio::co_spawn(
        mgr_strand_,
        [this, &client_options]() -> Awaitable<std::shared_ptr<AsioTcpClient>> {
          if (state_.load() != State::kStart) [[unlikely]] {
            AIMRT_WARN("Tcp cli pool is closed, will not return cli instance.");
            co_return std::shared_ptr<AsioTcpClient>();
          }

          const size_t client_hash =
              std::hash<boost::asio::ip::tcp::endpoint>{}(client_options.svr_ep);

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
                                   "Tcp client num reach the upper limit.");
          }

          auto client_ptr = std::make_shared<AsioTcpClient>(io_ptr_);
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

  // 日志打印句柄
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  // 配置
  Options options_;

  // 状态
  std::atomic<State> state_ = State::kPreInit;

  // client管理
  std::unordered_map<size_t, std::shared_ptr<AsioTcpClient>> client_map_;
};

}  // namespace aimrt::common::net
