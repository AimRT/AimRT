// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <boost/asio.hpp>
#include <boost/asio/experimental/awaitable_operators.hpp>

#include "boost/asio/detached.hpp"
#include "grpc_plugin/global.h"  // IWYU pragma: keep
#include "grpc_plugin/http2/request.h"
#include "grpc_plugin/http2/response.h"
#include "grpc_plugin/http2/server_session.h"
#include "grpc_plugin/server/options.h"
#include "net/http_dispatcher.h"
#include "util/string_util.h"

namespace aimrt::plugins::grpc_plugin::server {

using IOCtx = boost::asio::io_context;
using Strand = boost::asio::strand<boost::asio::io_context::executor_type>;
using Timer = boost::asio::steady_timer;

template <class T>
using Awaitable = boost::asio::awaitable<T>;

using Request = http2::Request;
using RequestPtr = std::shared_ptr<Request>;

using Response = http2::Response;
using ResponsePtr = std::shared_ptr<Response>;

using HttpHandle = std::function<Awaitable<void>(const RequestPtr&, ResponsePtr&)>;

namespace net = aimrt::common::net;
using Dispatcher = net::HttpDispatcher<Awaitable<void>(const RequestPtr&, ResponsePtr&)>;

namespace asio = boost::asio;

class Connection : public std::enable_shared_from_this<Connection> {
 public:
  Connection(const std::shared_ptr<IOCtx>& io_ptr,
             const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr,
             const std::shared_ptr<Dispatcher>& dispatcher_ptr)
      : io_ptr_(io_ptr),
        socket_strand_(asio::make_strand(*io_ptr)),
        socket_(socket_strand_),
        mgr_strand_(asio::make_strand(*io_ptr)),
        timer_(socket_strand_),
        logger_ptr_(logger_ptr),
        dispatcher_ptr_(dispatcher_ptr) {}

  ~Connection() = default;

  Connection(const Connection&) = delete;
  Connection& operator=(const Connection&) = delete;

  void Initialize(const std::shared_ptr<ConnectionOptions>& options_ptr) {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, ConnectionState::kInit) == ConnectionState::kPreInit,
        "Method can only be called when state is 'PreInit'");

    options_ptr_ = options_ptr;
    nghttp2_session_.InitSession(options_ptr->http2_settings);
  }

  void Start() {
    AIMRT_CHECK_ERROR_THROW(
        std::atomic_exchange(&state_, ConnectionState::kStart) == ConnectionState::kInit,
        "Method can only be called when state is 'Init'");

    remote_addr_ = aimrt::common::util::SSToString(socket_.remote_endpoint());

    auto self = this->shared_from_this();

    asio::co_spawn(
        socket_strand_,
        [this, self]() -> Awaitable<void> {
          try {
            while (state_.load() == ConnectionState::kStart && !close_connect_flag_) {
              co_await ReceiveFromRemote();

              auto full_request_list = nghttp2_session_.GetFullRequestList();
              if (full_request_list.empty()) {
                co_await SendToRemote();  // Send some http2 data to client
                continue;
              }

              tick_has_data_ = true;

              for (auto&& req_ptr_ref : full_request_list) {
                auto req_ptr = std::move(req_ptr_ref);
                auto url = req_ptr->GetUrl();
                const auto& handle = dispatcher_ptr_->GetHttpHandle(url.path);

                auto rsp_ptr = std::make_shared<Response>();
                rsp_ptr->SetStreamId(req_ptr->GetStreamId());
                rsp_ptr->AddHeader("content-type", "application/grpc");

                if (!handle) {
                  AIMRT_WARN("Unregistered URL: {}", url.path);
                  rsp_ptr->SetHttpStatus(http2::HttpStatus::kNotFound);
                  rsp_ptr->Write("Not Found");
                  nghttp2_session_.SubmitResponse(rsp_ptr);
                  co_await SendToRemote();
                  continue;
                }

                req_ptr->AddHeader("Remote-Endpoint", remote_addr_);

                asio::co_spawn(
                    io_ptr_->get_executor(),
                    [this, self, req_ptr = std::move(req_ptr), rsp_ptr = std::move(rsp_ptr), handle]() mutable -> Awaitable<void> {
                      // Send request to thread pool
                      co_await asio::post(asio::bind_executor(io_ptr_->get_executor(), asio::use_awaitable));
                      co_await handle(req_ptr, rsp_ptr);

                      // Submit response to http2 session in socket_strand_
                      co_await asio::post(asio::bind_executor(socket_strand_, asio::use_awaitable));
                      nghttp2_session_.SubmitResponse(rsp_ptr);
                      co_await SendToRemote();
                    },
                    asio::detached);
              }
              full_request_list.clear();
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE("Http svr session get exception and exit, remote addr {}, exception info: {}",
                        RemoteAddr(), e.what());
          }

          Shutdown();

          co_return;
        },
        asio::detached);

    asio::co_spawn(
        mgr_strand_,
        [this, self]() -> Awaitable<void> {
          try {
            while (state_.load() == ConnectionState::kStart) {
              timer_.expires_after(options_ptr_->max_no_data_duration);
              co_await timer_.async_wait(asio::use_awaitable);

              if (!tick_has_data_) {
                AIMRT_TRACE(
                    "Http svr session exit due to timeout ({} ms), remote addr {}",
                    std::chrono::duration_cast<std::chrono::milliseconds>(options_ptr_->max_no_data_duration).count(),
                    RemoteAddr());
                close_connect_flag_ = true;
                break;
              }
              tick_has_data_ = false;
            }
          } catch (const std::exception& e) {
            AIMRT_TRACE("Http svr session mgr get exception and exit, remote addr {}, exception info: {}",
                        RemoteAddr(), e.what());
          }

          Shutdown();

          co_return;
        },
        asio::detached);
  }

  void Shutdown() {
    if (std::atomic_exchange(&state_, ConnectionState::kShutdown) == ConnectionState::kShutdown) {
      return;
    }

    auto self = this->shared_from_this();
    asio::dispatch(socket_strand_, [this, self]() {
      try {
        close_connect_flag_ = true;
        socket_.shutdown(Tcp::socket::shutdown_both);
        socket_.cancel();
        socket_.close();
      } catch (const std::exception& e) {
        AIMRT_TRACE("Http svr session shutdown failed, remote addr {}, exception info: {}",
                    RemoteAddr(), e.what());
      }
    });

    asio::dispatch(mgr_strand_, [this, self]() {
      try {
        timer_.cancel();
      } catch (const std::exception& e) {
        AIMRT_TRACE("Http svr session mgr shutdown failed, remote addr {}, exception info: {}",
                    RemoteAddr(), e.what());
      }
    });
  }

  const aimrt::common::util::LoggerWrapper& GetLogger() const noexcept {
    return *logger_ptr_;
  }

  std::string_view RemoteAddr() const noexcept {
    return remote_addr_;
  }

  Tcp::socket& Socket() noexcept {
    return socket_;
  }

  bool IsRunning() const noexcept {
    return state_.load() == ConnectionState::kStart;
  }

 private:
  Awaitable<void> ReceiveFromRemote() {
    std::array<uint8_t, 65536> buffer;
    auto nread = co_await socket_.async_read_some(asio::buffer(buffer), asio::use_awaitable);

#ifndef NDEBUG
    std::string data_hex;
    for (size_t i = 0; i < nread; ++i) {
      data_hex += fmt::format("{:02x} ", buffer[i]);
    }
    std::string data_str;
    for (size_t i = 0; i < nread; ++i) {
      data_str += fmt::format("{:2c} ", isprint(buffer[i]) ? buffer[i] : '.');
    }
    AIMRT_TRACE("Http svr session get data, remote addr {}, data: \n{}\n{}", RemoteAddr(), data_hex, data_str);
#endif

    auto recv_ok = nghttp2_session_.ParseRecvMessage(
        std::string_view(reinterpret_cast<char*>(buffer.data()), nread));
    if (recv_ok != 0) {
      AIMRT_ERROR("Http svr session parse recv response failed, remote addr {}, parse failed", RemoteAddr());
      throw std::runtime_error("Http svr session parse recv response failed");
    }
  }

  Awaitable<void> SendToRemote() {
    http2::SimpleBuffer send_buffer(8192);
    auto send_ok = nghttp2_session_.GetSendMessage(send_buffer);
    if (send_ok != 0) {
      AIMRT_ERROR("Http svr session serialize send response failed, remote addr {}, send failed", RemoteAddr());
      throw std::runtime_error("Http svr session serialize send response failed");
    }
    if (!send_buffer.Empty()) {
      co_await asio::async_write(socket_,
                                 asio::buffer(send_buffer.GetStringView()),
                                 asio::use_awaitable);
    }
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

  std::shared_ptr<Dispatcher> dispatcher_ptr_;

  std::shared_ptr<ConnectionOptions> options_ptr_;
  std::atomic<ConnectionState> state_ = ConnectionState::kPreInit;

  std::string remote_addr_;
  std::atomic<bool> tick_has_data_ = false;
  bool close_connect_flag_ = false;

  http2::ServerSession nghttp2_session_;
};

}  // namespace aimrt::plugins::grpc_plugin::server
