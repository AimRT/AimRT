// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <boost/asio.hpp>
#include <boost/asio/experimental/awaitable_operators.hpp>

#include "grpc_plugin/client/options.h"
#include "grpc_plugin/global.h"  // IWYU pragma: keep
#include "grpc_plugin/http2/client_session.h"
#include "grpc_plugin/http2/request.h"
#include "grpc_plugin/http2/response.h"
#include "net/http_dispatcher.h"
#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::plugins::grpc_plugin::client {

using IOCtx = boost::asio::io_context;
using Tcp = boost::asio::ip::tcp;
using Strand = boost::asio::strand<IOCtx::executor_type>;
using Timer = boost::asio::steady_timer;

namespace asio = boost::asio;

template <class T>
using Awaitable = asio::awaitable<T>;

using Request = http2::Request;
using RequestPtr = std::shared_ptr<Request>;

using Response = http2::Response;
using ResponsePtr = std::shared_ptr<Response>;

using HttpHandleType = Awaitable<void>(const RequestPtr&, ResponsePtr&);

using HttpHandle = std::function<HttpHandleType>;

namespace net = aimrt::runtime::common::net;
using Dispatcher = net::HttpDispatcher<HttpHandleType>;

namespace chrono = std::chrono;
namespace asio = boost::asio;

using asio::experimental::awaitable_operators::operator||;

class Connection : public std::enable_shared_from_this<Connection> {
 public:
  Connection(const std::shared_ptr<boost::asio::io_context>& io_ptr,
             const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr)
      : io_ptr_(io_ptr),
        socket_strand_(asio::make_strand(*io_ptr)),
        socket_(socket_strand_),
        mgr_strand_(asio::make_strand(*io_ptr)),
        timer_(socket_strand_),
        logger_ptr_(logger_ptr) {}

  ~Connection() = default;

  Connection(const Connection&) = delete;
  Connection& operator=(const Connection&) = delete;

  void Initialize(std::shared_ptr<const ConnectionOptions>& options_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, ConnectionState::kInit) == ConnectionState::kPreInit,
        "Method can only be called when state is 'PreInit'.");

    options_ptr_ = options_ptr;
    nghttp2_session_.InitSession(options_ptr->http2_settings);
  }

  void Start() {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, ConnectionState::kStart) == ConnectionState::kInit,
        "Method can only be called when state is 'Init'.");

    auto self = this->shared_from_this();

    boost::asio::co_spawn(
        mgr_strand_,
        [this, self]() -> Awaitable<void> {
          try {
            while (state_.load() == ConnectionState::kStart) {
              timer_.expires_after(options_ptr_->max_no_data_duration);
              co_await timer_.async_wait(boost::asio::use_awaitable);

              if (!tick_has_data_) {
                AIMRT_TRACE("Http2 cli session exit due to timeout({} ms), remote addr {}",
                            chrono::duration_cast<chrono::milliseconds>(options_ptr_->max_no_data_duration).count(),
                            RemoteAddr());
                break;
              }

              tick_has_data_ = false;
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE("Http2 cli session got exception and exit, remote addr {}, exception info: {}",
                        RemoteAddr(), e.what());
          }

          Shutdown();

          co_return;
        },
        boost::asio::detached);
  }

  void Shutdown() {
    if (std::atomic_exchange(&state_, ConnectionState::kShutdown) == ConnectionState::kShutdown)
      return;

    auto self = shared_from_this();
    boost::asio::dispatch(socket_strand_, [this, self]() {
      try {
        socket_.shutdown(Tcp::socket::shutdown_both);
        socket_.cancel();
        socket_.close();
      } catch (const std::exception& e) {
        AIMRT_TRACE("Http cli session shutdown failed, remote addr {}, exception info: {}",
                    RemoteAddr(), e.what());
      }
    });

    boost::asio::dispatch(mgr_strand_, [this, self]() {
      try {
        timer_.cancel();
      } catch (const std::exception& e) {
        AIMRT_TRACE("Http cli session mgr shutdown failed, remote addr {}, exception info: {}",
                    RemoteAddr(), e.what());
      }
    });
  }

  Awaitable<ResponsePtr> HttpSendRecvCo(const RequestPtr& req_ptr,
                                        std::chrono::nanoseconds timeout = std::chrono::seconds(5)) {
    return boost::asio::co_spawn(
        socket_strand_,
        [this, &req_ptr, timeout]() -> Awaitable<ResponsePtr> {
          AIMRT_CHECK_ERROR_THROW(
              state_.load() == ConnectionState::kStart,
              "Method can only be called when state is 'Start'.");

          try {
            auto start_time_point = chrono::steady_clock::now();
            chrono::nanoseconds cur_duration;

            if (first_time_entry_) [[unlikely]] {
              first_time_entry_ = false;

              // resolve
              asio::ip::tcp::resolver resolver(socket_strand_);
              auto const dst = co_await resolver.async_resolve(
                  options_ptr_->host,
                  options_ptr_->service,
                  asio::use_awaitable);

              cur_duration = chrono::steady_clock::now() - start_time_point;
              auto connect_timeout = timeout - cur_duration;
              AIMRT_CHECK_ERROR_THROW(connect_timeout > chrono::nanoseconds::zero(),
                                      "Timeout before connection attempt.");

              asio::steady_timer connect_timer(socket_strand_, connect_timeout);

              co_await (asio::async_connect(socket_, dst, asio::use_awaitable) ||
                        connect_timer.async_wait(asio::use_awaitable));

              AIMRT_CHECK_ERROR_THROW(connect_timer.expiry() >= chrono::steady_clock::now(),
                                      "Timeout for connection attempt.");
              AIMRT_CHECK_ERROR_THROW(socket_.is_open(), "Failed to connect to server.");

              remote_addr_ = aimrt::common::util::SSToString(socket_.remote_endpoint());
              AIMRT_TRACE("Http2 cli session async connect, remote addr {}.", RemoteAddr());
            }

            // write
            cur_duration = chrono::steady_clock::now() - start_time_point;
            AIMRT_CHECK_ERROR_THROW(cur_duration < timeout, "Timeout.");

            AIMRT_TRACE("Http2 cli session async write, remote addr {}, timeout {}ms.",
                        RemoteAddr(),
                        chrono::duration_cast<chrono::milliseconds>(timeout - cur_duration).count());
            int submit_ok = nghttp2_session_.SubmitRequest(req_ptr);
            AIMRT_CHECK_ERROR_THROW(submit_ok == 0, "Failed to submit request.");
            http2::SimpleBuffer send_buf(8192);
            nghttp2_session_.GetSendMessage(send_buf);
            if (!send_buf.Empty()) {
              auto write_timer = asio::steady_timer(socket_strand_, timeout - cur_duration);
              auto write_result = co_await (asio::async_write(socket_,
                                                              asio::buffer(send_buf.GetStringView()),
                                                              asio::use_awaitable) ||
                                            write_timer.async_wait(asio::use_awaitable));
              AIMRT_CHECK_ERROR_THROW(write_timer.expiry() >= chrono::steady_clock::now(),
                                      "Timeout for write.");
              size_t nwrite = std::get<0>(write_result);
              AIMRT_TRACE("Http2 cli session write {} bytes to {}.", nwrite, RemoteAddr());
              tick_has_data_ = true;
            }

            // read
            cur_duration = chrono::steady_clock::now() - start_time_point;
            AIMRT_CHECK_ERROR_THROW(cur_duration < timeout, "Timeout.");

            // FIXME(zhangyi): The read buffer size is fixed
            while (nghttp2_session_.ResponseListEmpty()) {
              std::array<uint8_t, 8192> buf;
              auto read_timer = asio::steady_timer(socket_strand_, timeout - cur_duration);
              auto read_result = co_await (socket_.async_read_some(asio::buffer(buf), asio::use_awaitable) ||
                                           read_timer.async_wait(asio::use_awaitable));
              AIMRT_CHECK_ERROR_THROW(read_timer.expiry() >= chrono::steady_clock::now(),
                                      "Timeout for read.");
              size_t nread = std::get<0>(read_result);
              AIMRT_TRACE("Http2 cli session read {} bytes from {}.", nread, RemoteAddr());
              tick_has_data_ = true;

              nghttp2_session_.ParseRecvMessage(std::string_view(reinterpret_cast<const char*>(buf.data()), nread));
            }

            idle_flag_ = true;

            auto rsp_list = nghttp2_session_.GetFullResponseList();

            auto rsp_list_size = std::distance(rsp_list.begin(), rsp_list.end());
            AIMRT_CHECK_ERROR_THROW(rsp_list_size == 1, "Response list size is not 1.");
            co_return rsp_list.front();
          } catch (const std::exception& e) {
            Shutdown();
            AIMRT_WARN_THROW("Http2 cli session send & recv failed and exit, remote addr {}, exception info: {}",
                             RemoteAddr(), e.what());
          }
        },
        boost::asio::use_awaitable);
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const {
    return *logger_ptr_;
  }

  bool CheckIdleAndUse() {
    bool is_idle = std::atomic_exchange(&idle_flag_, false);
    return is_idle && state_ == ConnectionState::kStart;
  }

  std::string_view RemoteAddr() const {
    return remote_addr_;
  }

  bool IsRunning() const {
    return state_ == ConnectionState::kStart;
  }

 private:
  enum class ConnectionState : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  std::shared_ptr<IOCtx> io_ptr_;
  Strand socket_strand_;
  Tcp::socket socket_;
  Strand mgr_strand_;
  Timer timer_;

  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::shared_ptr<const ConnectionOptions> options_ptr_;

  std::atomic<ConnectionState> state_ = ConnectionState::kPreInit;

  std::atomic_bool idle_flag_ = false;
  std::string remote_addr_;
  std::atomic_bool tick_has_data_ = false;
  bool first_time_entry_ = true;

  http2::ClientSession nghttp2_session_;
};

}  // namespace aimrt::plugins::grpc_plugin::client