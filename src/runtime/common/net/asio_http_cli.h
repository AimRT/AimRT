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

namespace aimrt::runtime::common::net {

class AsioHttpClient : public std::enable_shared_from_this<AsioHttpClient> {
 public:
  using IOCtx = boost::asio::io_context;
  using Tcp = boost::asio::ip::tcp;
  using Strand = boost::asio::strand<IOCtx::executor_type>;
  using Timer = boost::asio::steady_timer;

  template <class T>
  using Awaitable = boost::asio::awaitable<T>;

  template <class Body>
  using Request = boost::beast::http::request<Body>;

  template <class Body>
  using Response = boost::beast::http::response<Body>;

  struct Options {
    /// 服务器域名或ip
    std::string host;

    /// 服务（如http、ftp）或端口号
    std::string service;

    /// 连接最长无数据时间
    std::chrono::nanoseconds max_no_data_duration = std::chrono::seconds(60);

    /// 最大连接数
    size_t max_session_num = 1000;

    /// 校验配置
    static Options Verify(const Options& verify_options) {
      Options options(verify_options);

      if (options.max_session_num < 1) options.max_session_num = 1;

      return options;
    }
  };

  explicit AsioHttpClient(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioHttpClient() = default;

  AsioHttpClient(const AsioHttpClient&) = delete;
  AsioHttpClient& operator=(const AsioHttpClient&) = delete;

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
      for (auto& session_ptr : session_ptr_list_)
        session_ptr->Shutdown();

      session_ptr_list_.clear();
    });
  }

  /**
   * @brief http请求协程接口
   *
   * @tparam ReqBodyType 请求包的body类型
   * @tparam RspBodyType 返回包的body类型
   * @param req 请求包
   * @param timeout 超时时间
   * @return 返回包
   */
  template <typename ReqBodyType = boost::beast::http::string_body,
            typename RspBodyType = boost::beast::http::string_body>
  Awaitable<Response<RspBodyType>> HttpSendRecvCo(
      const Request<ReqBodyType>& req,
      std::chrono::nanoseconds timeout = std::chrono::seconds(5)) {
    return boost::asio::co_spawn(
        mgr_strand_,
        [this, &req, timeout]() -> Awaitable<Response<RspBodyType>> {
          AIMRT_CHECK_WARN_THROW(
              state_.load() == State::kStart,
              "Http cli is closed, will not send request.");

          // 找可用session，没有就新建一个。同时清理已失效session
          std::shared_ptr<Session> session_ptr;

          for (auto itr = session_ptr_list_.begin(); itr != session_ptr_list_.end();) {
            if ((*itr)->IsRunning()) {
              if ((*itr)->CheckIdleAndUse()) {
                session_ptr = *itr;
                break;
              }
              ++itr;
            } else {
              session_ptr_list_.erase(itr++);
            }
          }

          if (!session_ptr) {
            AIMRT_CHECK_ERROR_THROW(session_ptr_list_.size() < options_.max_session_num,
                                    "Http client session num reach the upper limit.");

            session_ptr = std::make_shared<Session>(io_ptr_, logger_ptr_);
            session_ptr->Initialize(session_options_ptr_);
            session_ptr->Start();

            session_ptr_list_.emplace_back(session_ptr);
          }

          co_return co_await session_ptr->HttpSendRecvCo<ReqBodyType, RspBodyType>(req, timeout);
        },
        boost::asio::use_awaitable);
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  bool IsRunning() const { return state_.load() == State::kStart; }

 private:
  struct SessionOptions {
    explicit SessionOptions(const Options& options)
        : host(options.host),
          service(options.service),
          max_no_data_duration(options.max_no_data_duration) {}

    std::string host;
    std::string service;
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
          stream_(session_socket_strand_),
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

      auto self = shared_from_this();

      // 定时器
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
                      "Http cli session exit due to timeout({} ms), remote addr {}.",
                      std::chrono::duration_cast<std::chrono::milliseconds>(session_options_ptr_->max_no_data_duration).count(),
                      RemoteAddr());
                  break;
                }
              }
            } catch (const std::exception& e) {
              AIMRT_TRACE(
                  "Http cli session timer get exception and exit, remote addr {}, exception info: {}",
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
                stream_.socket().shutdown(Tcp::socket::shutdown_both);
                ++stop_step;
              case 2:
                stream_.socket().cancel();
                ++stop_step;
              case 3:
                stream_.socket().close();
                ++stop_step;
              case 4:
                stream_.socket().release();
                ++stop_step;
              case 5:
                stream_.cancel();
                ++stop_step;
              case 6:
                stream_.close();
                ++stop_step;
              case 7:
                stream_.release_socket();
                ++stop_step;
              default:
                stop_step = 0;
                break;
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE(
                "Http cli session stop get exception at step {}, remote addr {}, exception info: {}",
                stop_step, RemoteAddr(), e.what());
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
                "Http cli session mgr stop get exception at step {}, remote addr {}, exception info: {}",
                stop_step, RemoteAddr(), e.what());
            ++stop_step;
          }
        }
      });
    }

    template <typename ReqBodyType = boost::beast::http::string_body,
              typename RspBodyType = boost::beast::http::string_body>
    Awaitable<Response<RspBodyType>> HttpSendRecvCo(
        const Request<ReqBodyType>& req, std::chrono::nanoseconds timeout) {
      return boost::asio::co_spawn(
          session_socket_strand_,
          [this, &req, timeout]() -> Awaitable<Response<RspBodyType>> {
            AIMRT_CHECK_WARN_THROW(
                state_.load() == SessionState::kStart,
                "Http cli session is closed, will not send request.");

            try {
              namespace chrono = std::chrono;
              namespace asio = boost::asio;
              namespace http = boost::beast::http;

              auto start_time_point = chrono::steady_clock::now();
              chrono::nanoseconds cur_duration;

              if (first_time_entry_) [[unlikely]] {
                first_time_entry_ = false;

                // resolve
                asio::ip::tcp::resolver resolver(session_socket_strand_);
                auto const dst = co_await resolver.async_resolve(
                    session_options_ptr_->host,
                    session_options_ptr_->service,
                    asio::use_awaitable);

                // connect
                cur_duration = chrono::steady_clock::now() - start_time_point;
                AIMRT_CHECK_ERROR_THROW(cur_duration < timeout, "Timeout.");

                stream_.expires_after(timeout - cur_duration);
                co_await stream_.async_connect(dst, asio::use_awaitable);

                remote_addr_ = aimrt::common::util::SSToString(stream_.socket().remote_endpoint());
                AIMRT_TRACE("Http cli session async connect, remote addr {}.", RemoteAddr());
              }

              // write
              cur_duration = chrono::steady_clock::now() - start_time_point;
              AIMRT_CHECK_ERROR_THROW(cur_duration < timeout, "Timeout.");

              AIMRT_TRACE(
                  "Http cli session async write, remote addr {}, timeout {}ms.",
                  RemoteAddr(),
                  chrono::duration_cast<chrono::milliseconds>(timeout - cur_duration).count());
              stream_.expires_after(timeout - cur_duration);
              size_t write_size = co_await http::async_write(stream_, req, asio::use_awaitable);
              AIMRT_TRACE("Http cli session write {} bytes to {}.", write_size, RemoteAddr());
              tick_has_data_ = true;

              // read
              cur_duration = chrono::steady_clock::now() - start_time_point;
              AIMRT_CHECK_ERROR_THROW(cur_duration < timeout, "Timeout.");

              AIMRT_TRACE(
                  "Http cli session async read, remote addr {}, timeout {}ms.",
                  RemoteAddr(),
                  chrono::duration_cast<chrono::milliseconds>(timeout - cur_duration).count());
              stream_.expires_after(timeout - cur_duration);
              http::response_parser<RspBodyType> rsp_parser;
              rsp_parser.eager(true);
              rsp_parser.body_limit(1024 * 1024 * 16);
              size_t read_size = co_await http::async_read(
                  stream_, buffer_, rsp_parser, asio::use_awaitable);
              AIMRT_TRACE("Http cli session read {} bytes from {}.", read_size, RemoteAddr());
              tick_has_data_ = true;
              Response<RspBodyType> rsp = rsp_parser.release();

              if (req.need_eof() || rsp.need_eof()) {
                AIMRT_TRACE("Http cli session close due to eof, remote addr {}.", RemoteAddr());
                Shutdown();
              }

              idle_flag_ = true;

              co_return rsp;

            } catch (const std::exception& e) {
              AIMRT_WARN(
                  "Http cli session send recv co get exception and exit, remote addr {}, exception info: {}",
                  RemoteAddr(), e.what());
            }

            Shutdown();

            AIMRT_WARN_THROW("Http client session send & recv failed and exit.");
          },
          boost::asio::use_awaitable);
    }

    const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

    bool CheckIdleAndUse() {
      bool is_idle = std::atomic_exchange(&idle_flag_, false);
      if (!is_idle || state_ != SessionState::kStart) return false;
      return true;
    }

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
    Strand session_mgr_strand_;
    Timer timer_;
    Strand session_socket_strand_;
    boost::beast::tcp_stream stream_;

    // 日志打印句柄
    std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

    // 配置
    std::shared_ptr<const SessionOptions> session_options_ptr_;

    // 状态
    std::atomic<SessionState> state_ = SessionState::kPreInit;

    // misc
    std::atomic_bool idle_flag_ = false;
    std::string remote_addr_;
    std::atomic_bool tick_has_data_ = false;
    boost::beast::flat_buffer buffer_;
    bool first_time_entry_ = true;
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

  // 配置
  Options options_;

  // 状态
  std::atomic<State> state_ = State::kPreInit;

  // session管理
  std::shared_ptr<const SessionOptions> session_options_ptr_;
  std::list<std::shared_ptr<Session>> session_ptr_list_;
};

class AsioHttpClientPool
    : public std::enable_shared_from_this<AsioHttpClientPool> {
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

  explicit AsioHttpClientPool(const std::shared_ptr<IOCtx>& io_ptr)
      : io_ptr_(io_ptr),
        mgr_strand_(boost::asio::make_strand(*io_ptr_)),
        logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}

  ~AsioHttpClientPool() = default;

  AsioHttpClientPool(const AsioHttpClientPool&) = delete;
  AsioHttpClientPool& operator=(const AsioHttpClientPool&) = delete;

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
   * @brief 获取http client
   * @note 如果http client目的地址相同，则会复用已有的http client
   * @param options http client的配置
   * @return http client
   */
  Awaitable<std::shared_ptr<AsioHttpClient>> GetClient(
      const AsioHttpClient::Options& client_options) {
    return boost::asio::co_spawn(
        mgr_strand_,
        [this, &client_options]() -> Awaitable<std::shared_ptr<AsioHttpClient>> {
          if (state_.load() != State::kStart) [[unlikely]] {
            AIMRT_WARN("Http cli pool is closed, will not return cli instance.");
            co_return std::shared_ptr<AsioHttpClient>();
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
                                   "Http client num reach the upper limit.");
          }

          auto client_ptr = std::make_shared<AsioHttpClient>(io_ptr_);
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

  // 日志打印句柄
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  // 配置
  Options options_;

  // 状态
  std::atomic<State> state_ = State::kPreInit;

  // client管理
  std::unordered_map<std::string, std::shared_ptr<AsioHttpClient>> client_map_;
};

}  // namespace aimrt::runtime::common::net
