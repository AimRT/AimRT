// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <sstream>
#include <string_view>

#include "grpc_plugin/http2/buffer.h"
#include "grpc_plugin/http2/header.h"
#include "grpc_plugin/http2/status.h"

namespace aimrt::plugins::grpc_plugin::http2 {

class Response {
 public:
  explicit Response(size_t capacity = 8192) : body_(capacity) {}

  void SetStreamId(int32_t stream_id) { stream_id_ = stream_id; }
  [[nodiscard]] int32_t GetStreamId() const { return stream_id_; }

  [[nodiscard]] const Headers& GetHeaders() const { return headers_; }

  [[nodiscard]] const SimpleBuffer& GetBody() const { return body_; }
  [[nodiscard]] SimpleBuffer& GetMutableBody() { return body_; }

  [[nodiscard]] const Trailers& GetTrailers() const { return trailers_; }

  void SetHttpStatus(HttpStatus status) { http_status_ = status; }
  [[nodiscard]] HttpStatus GetHttpStatus() const { return http_status_; }

  void AddHeader(std::string_view name, std::string_view value) {
    headers_.emplace(name, value);
  }

  void AddTrailer(std::string_view name, std::string_view value) {
    trailers_.emplace(name, value);
  }

  void Write(const uint8_t* data, size_t len) {
    body_.Write(data, len);
  }

  void Write(std::string_view data) {
    body_.Write(reinterpret_cast<const uint8_t*>(data.data()), data.size());
  }

  [[nodiscard]] std::string ToString() const {
    std::ostringstream ss;
    ss << "Response{"
       << "stream_id=" << stream_id_
       << ", http_status=" << static_cast<int>(http_status_)
       << ", headers={";
    for (const auto& [name, value] : headers_) {
      ss << ", " << name << "=" << value;
    }
    ss << "}, trailers={";
    for (const auto& [name, value] : trailers_) {
      ss << ", " << name << "=" << value;
    }
    ss << "}, body={" << body_.GetStringView() << "}}";
    return ss.str();
  }

 private:
  int32_t stream_id_ = -1;
  Headers headers_;
  Trailers trailers_;
  SimpleBuffer body_;
  HttpStatus http_status_ = HttpStatus::kOk;
};
using ResponsePtr = std::shared_ptr<Response>;

}  // namespace aimrt::plugins::grpc_plugin::http2