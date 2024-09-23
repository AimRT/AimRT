// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <chrono>

#include <boost/asio.hpp>

#include "grpc_plugin/http2/session.h"

namespace aimrt::plugins::grpc_plugin::server {

using Tcp = boost::asio::ip::tcp;

struct ServerOptions {
  Tcp::endpoint ep = Tcp::endpoint{boost::asio::ip::address_v4(), 50050};

  size_t max_connection_num = 1000000;

  std::chrono::nanoseconds mgr_timer_dt = std::chrono::seconds(5);

  std::chrono::nanoseconds max_no_data_duration = std::chrono::seconds(60);

  static ServerOptions Verify(const ServerOptions& verify_options) {
    ServerOptions options(verify_options);

    if (options.max_connection_num < 1) options.max_connection_num = 1;

    if (options.max_connection_num > Tcp::acceptor::max_listen_connections)
      options.max_connection_num = Tcp::acceptor::max_listen_connections;

    if (options.mgr_timer_dt < std::chrono::milliseconds(100))
      options.mgr_timer_dt = std::chrono::milliseconds(100);

    if (options.max_no_data_duration < std::chrono::seconds(10))
      options.max_no_data_duration = std::chrono::seconds(10);

    return options;
  }

  http2::Session::Http2Settings http2_settings;
};

struct ConnectionOptions {
  explicit ConnectionOptions(const ServerOptions& options)
      : max_no_data_duration(options.max_no_data_duration),
        http2_settings(options.http2_settings) {}

  std::chrono::nanoseconds max_no_data_duration;
  http2::Session::Http2Settings http2_settings;
};

}  // namespace aimrt::plugins::grpc_plugin::server
