// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "grpc_plugin/http2/session.h"

#include <queue>

#include <nghttp2/nghttp2.h>

#include "grpc_plugin/global.h"  // IWYU pragma: keep
#include "grpc_plugin/http2/buffer.h"

namespace aimrt::plugins::grpc_plugin::http2 {
// The following source code is from trpc-cpp.
// Copied and modified from
// https://github.com/trpc-group/trpc-cpp/blob/v1.2.0/trpc/codec/grpc/http2/session.cc

Session::~Session() {
  if (session_) {
    nghttp2_session_del(session_);
    session_ = nullptr;
  }
}

StreamPtr Session::CreateStream() { return std::make_shared<Stream>(); }

StreamPtr Session::FindStream(int32_t stream_id) {
  auto it = streams_.find(stream_id);
  if (it == streams_.end()) {
    return nullptr;
  }
  return it->second;
}

void Session::AddStream(StreamPtr&& stream_ptr) {
  streams_.emplace(stream_ptr->GetStreamId(), std::move(stream_ptr));
}

void Session::RemoveStream(int32_t stream_id) {
  auto it = streams_.find(stream_id);
  if (it != streams_.end()) {
    streams_.erase(it);
  }
}

int Session::SubmitHeader(int32_t stream_id, const Headers& headers) {
  auto stream = FindStream(stream_id);
  if (!stream) {
    AIMRT_ERROR("stream not found, stream id: {}", stream_id);
    return -1;
  }

  std::vector<nghttp2_nv> nva;
  nva.reserve(headers.size());
  for (const auto& [name, value] : headers) {
    nva.push_back(nghttp2_nv{
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(name.data())),
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(value.data())),
        name.size(),
        value.size(),
        NGHTTP2_NV_FLAG_NO_COPY_NAME | NGHTTP2_NV_FLAG_NO_COPY_VALUE,
    });
  }

  auto submit_ok = nghttp2_submit_headers(session_, NGHTTP2_FLAG_NONE, stream_id, nullptr, nva.data(), nva.size(), nullptr);
  if (submit_ok != 0) {
    AIMRT_ERROR("nghttp2_submit_headers error: {}, {}", submit_ok, nghttp2_strerror(submit_ok));
    return -1;
  }
  return 0;
}

int Session::SubmitData(int32_t stream_id, SimpleBuffer&& buffer) {
  auto stream = FindStream(stream_id);
  if (!stream) {
    AIMRT_ERROR("stream not found, stream id: {}", stream_id);
    return -1;
  }

  stream_data_queue_.push(std::move(buffer));

  nghttp2_data_provider data_provider{};
  data_provider.source.ptr = &stream_data_queue_;
  data_provider.read_callback = [](nghttp2_session* session, int32_t stream_id, uint8_t* buf, size_t len,
                                   uint32_t* data_flags, nghttp2_data_source* source, void* user_data) -> ssize_t {
    auto* stream_data_queue = static_cast<std::queue<SimpleBuffer>*>(source->ptr);
    if (stream_data_queue->empty()) {
      *data_flags |= NGHTTP2_DATA_FLAG_EOF | NGHTTP2_DATA_FLAG_NO_END_STREAM;
      return 0;
    }

    auto& content_buffer = stream_data_queue->front();
    auto content_buffer_size = content_buffer.GetReadableSize();
    if (content_buffer_size == 0) {
      *data_flags |= NGHTTP2_DATA_FLAG_EOF | NGHTTP2_DATA_FLAG_NO_END_STREAM;
      stream_data_queue->pop();
      return 0;
    }

    size_t bytes_to_send = std::min(len, content_buffer_size);
    content_buffer.Read(buf, bytes_to_send);
    content_buffer.Consume(bytes_to_send);
    return static_cast<ssize_t>(bytes_to_send);
  };

  auto submit_ok = nghttp2_submit_data(session_, NGHTTP2_FLAG_NONE, stream_id, &data_provider);
  if (submit_ok != 0) {
    AIMRT_ERROR("nghttp2_submit_data error: {}, {}", submit_ok, nghttp2_strerror(submit_ok));
    return -1;
  }
  return 0;
}

int Session::SubmitTrailer(int32_t stream_id, const Trailers& trailers) {
  auto stream = FindStream(stream_id);
  if (!stream) {
    AIMRT_ERROR("stream not found, stream id: {}", stream_id);
    return -1;
  }

  std::vector<nghttp2_nv> nva;
  nva.reserve(trailers.size());
  for (const auto& [name, value] : trailers) {
    nva.push_back(nghttp2_nv{
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(name.data())),
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(value.data())),
        name.size(),
        value.size(),
        NGHTTP2_NV_FLAG_NO_COPY_NAME | NGHTTP2_NV_FLAG_NO_COPY_VALUE,
    });
  }

  auto submit_ok = nghttp2_submit_trailer(session_, stream_id, nva.data(), nva.size());
  if (submit_ok != 0) {
    AIMRT_ERROR("nghttp2_submit_trailer error: {}, {}", submit_ok, nghttp2_strerror(submit_ok));
    return -1;
  }
  return 0;
}

int Session::SubmitReset(int32_t stream_id, uint32_t error_code) {
  auto submit_ok = nghttp2_submit_rst_stream(session_, NGHTTP2_FLAG_NONE, stream_id, error_code);
  if (submit_ok != 0) {
    AIMRT_ERROR("nghttp2_submit_rst_stream error: {}, {}", submit_ok, nghttp2_strerror(submit_ok));
    return -1;
  }
  return 0;
}

int Session::ParseRecvMessage(std::string_view in) {
  const auto* data = reinterpret_cast<const uint8_t*>(in.data());
  auto len = in.size();

  ssize_t pos = 0;
  while (pos < len) {
    auto nread = nghttp2_session_mem_recv(session_, data + pos, len - pos);
    if (nread <= 0) {
      AIMRT_DEBUG("nghttp2_session_mem_recv error: {}, {}", nread, nghttp2_strerror(nread));
      return -1;
    }
    pos += nread;
  }

  return 0;
}

int Session::GetSendMessage(SimpleBuffer& out) {
  while (true) {
    const uint8_t* buffer = nullptr;
    ssize_t nwrite = nghttp2_session_mem_send(session_, &buffer);
    if (nwrite < 0) {
      AIMRT_ERROR("nghttp2_session_mem_send error: {}, {}", nwrite, nghttp2_strerror(nwrite));
      return -1;
    }

    // EOF
    if (nwrite == 0) {
      return 0;
    }

    out.Write(buffer, nwrite);
  }

  return 0;
}
// End of source code from trpc-cpp.

}  // namespace aimrt::plugins::grpc_plugin::http2
