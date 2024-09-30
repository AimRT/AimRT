// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <nghttp2/nghttp2.h>
#include <memory>
#include <queue>
#include <unordered_map>

#include "grpc_plugin/http2/buffer.h"
#include "grpc_plugin/http2/header.h"

namespace aimrt::plugins::grpc_plugin::http2 {
// The following source code is from trpc-cpp.
// Copied and modified from
// https://github.com/trpc-group/trpc-cpp/blob/v1.2.0/trpc/codec/grpc/http2/session.h

class Request;
using RequestPtr = std::shared_ptr<Request>;

class Response;
using ResponsePtr = std::shared_ptr<Response>;

class Session;
using SessionPtr = std::shared_ptr<Session>;

class Stream {
 public:
  [[nodiscard]] int32_t GetStreamId() const { return stream_id_; }
  void SetStreamId(int32_t stream_id) { stream_id_ = stream_id; }

  void SetSession(Session* session) { session_ = session; }
  [[nodiscard]] Session* GetSession() const { return session_; }

  void SetRequest(const RequestPtr& request_ptr) { request_ptr_ = request_ptr; }
  [[nodiscard]] const RequestPtr& GetRequest() const { return request_ptr_; }
  RequestPtr GetMutableRequest() { return std::move(request_ptr_); }

  void SetResponse(const ResponsePtr& response_ptr) { response_ptr_ = response_ptr; }
  [[nodiscard]] const ResponsePtr& GetResponse() const { return response_ptr_; }
  ResponsePtr GetMutableResponse() { return std::move(response_ptr_); }

 private:
  int32_t stream_id_ = 0;
  Session* session_ = nullptr;
  RequestPtr request_ptr_ = nullptr;
  ResponsePtr response_ptr_ = nullptr;
};
using StreamPtr = std::shared_ptr<Stream>;

class Session {
 public:
  struct Http2Settings {
    uint32_t max_concurrent_streams = 100;
    uint32_t initial_window_size = (1U << 31) - 1;
  };

 public:
  virtual ~Session();

  virtual int InitSession(Http2Settings settings) = 0;

  virtual StreamPtr CreateStream();
  virtual StreamPtr FindStream(int32_t stream_id);
  virtual void AddStream(StreamPtr&& stream_ptr);
  virtual void RemoveStream(int32_t stream_id);

  virtual int SubmitRequest(const RequestPtr& request_ptr) { return -1; }
  virtual int SubmitResponse(const ResponsePtr& response_ptr) { return -1; }

  virtual int SubmitHeader(int32_t stream_id, const Headers& headers);
  virtual int SubmitData(int32_t stream_id, SimpleBuffer&& buffer);
  virtual int SubmitTrailer(int32_t stream_id, const Trailers& trailers);
  virtual int SubmitReset(int32_t stream_id, uint32_t error_code);

  virtual int ParseRecvMessage(std::string_view in);
  virtual int GetSendMessage(SimpleBuffer& out);

 protected:
  Http2Settings settings_;
  nghttp2_session* session_ = nullptr;
  std::unordered_map<int32_t, StreamPtr> streams_;
  std::queue<SimpleBuffer> stream_data_queue_;
};
// End of source code from trpc-cpp.

}  // namespace aimrt::plugins::grpc_plugin::http2