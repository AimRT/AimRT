// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <string>

#include "grpc_plugin/http2/session.h"

namespace aimrt::plugins::grpc_plugin::client {

struct ClientOptions {
  std::string host;
  std::string service;
  std::chrono::nanoseconds max_no_data_duration = std::chrono::seconds(60);
  size_t max_connection_num = 10;
  http2::Session::Http2Settings http2_settings;

  static ClientOptions Verify(const ClientOptions& verify_options) {
    ClientOptions options(verify_options);

    if (options.max_connection_num < 1) options.max_connection_num = 1;

    return options;
  }
};

struct ConnectionOptions {
  explicit ConnectionOptions(const ClientOptions& options)
      : max_no_data_duration(options.max_no_data_duration),
        host(options.host),
        service(options.service),
        http2_settings(options.http2_settings) {}

  std::string host;
  std::string service;
  std::chrono::nanoseconds max_no_data_duration;
  http2::Session::Http2Settings http2_settings;
};

struct ClientPoolOptions {
  size_t max_client_num = 1000;

  static ClientPoolOptions Verify(const ClientPoolOptions& verify_options) {
    ClientPoolOptions options(verify_options);

    if (options.max_client_num < 10) options.max_client_num = 10;

    return options;
  }
};

}  // namespace aimrt::plugins::grpc_plugin::client
