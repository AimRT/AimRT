// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "grpc_plugin/http2/server_session.h"

#include <nghttp2/nghttp2.h>

#include <string>

#include "grpc_plugin/global.h"  // IWYU pragma: keep
#include "grpc_plugin/http2/http2.h"
#include "grpc_plugin/http2/request.h"
#include "grpc_plugin/http2/response.h"  // IWYU pragma: keep
#include "util/deferred.h"
#include "util/url_parser.h"

namespace aimrt::plugins::grpc_plugin::http2 {
// The following source code is from trpc-cpp.
// Copied and modified from
// https://github.com/trpc-group/trpc-cpp/blob/v1.2.0/trpc/codec/grpc/http2/server_session.cc

namespace {

int OnBeginHeadersCallback(nghttp2_session* session, const nghttp2_frame* frame, void* user_data);

int OnHeaderCallback(nghttp2_session* session, const nghttp2_frame* frame, const uint8_t* name, size_t name_len,
                     const uint8_t* value, size_t value_len, uint8_t flags, void* user_data);

int OnFrameRecvCallback(nghttp2_session* session, const nghttp2_frame* frame, void* user_data);

int OnDataChunkRecvCallback(nghttp2_session* session, uint8_t flags, int32_t stream_id, const uint8_t* data, size_t len,
                            void* user_data);

int OnStreamCloseCallback(nghttp2_session* session, int32_t stream_id, uint32_t error_code, void* user_data);

int OnFrameNotSendCallback(nghttp2_session* session, const nghttp2_frame* frame, int32_t error_code, void* user_data);

}  // namespace

int ServerSession::InitSession(Http2Settings settings) {
  nghttp2_session_callbacks* callbacks = nullptr;
  auto new_ok = nghttp2_session_callbacks_new(&callbacks);
  if (new_ok != 0) {
    AIMRT_ERROR("nghttp2_session_callbacks_new failed: {}", new_ok);
    return -1;
  }

  auto callbacks_defer_deleter =
      common::util::Deferred([callbacks]() { nghttp2_session_callbacks_del(callbacks); });

  nghttp2_session_callbacks_set_on_begin_headers_callback(callbacks, OnBeginHeadersCallback);
  nghttp2_session_callbacks_set_on_header_callback(callbacks, OnHeaderCallback);
  nghttp2_session_callbacks_set_on_frame_recv_callback(callbacks, OnFrameRecvCallback);
  nghttp2_session_callbacks_set_on_data_chunk_recv_callback(callbacks, OnDataChunkRecvCallback);
  nghttp2_session_callbacks_set_on_stream_close_callback(callbacks, OnStreamCloseCallback);
  nghttp2_session_callbacks_set_on_frame_not_send_callback(callbacks, OnFrameNotSendCallback);

  new_ok = nghttp2_session_server_new(&session_, callbacks, this);
  if (new_ok != 0) {
    AIMRT_ERROR("nghttp2_session_server_new failed: {}", new_ok);
    return -1;
  }

  auto submit_ok = SubmitSettings(settings);
  if (!submit_ok) {
    return -1;
  }

  return 0;
}

int ServerSession::SubmitResponse(const ResponsePtr& response_ptr) {
  auto stream_id = response_ptr->GetStreamId();
  auto stream = FindStream(stream_id);
  if (!stream) {
    return -1;
  }

  stream->SetResponse(response_ptr);

  const auto& headers = response_ptr->GetHeaders();

  std::vector<nghttp2_nv> nva;
  nva.reserve(headers.size() + 1);
  auto status_str = std::to_string(static_cast<uint32_t>(response_ptr->GetHttpStatus()));
  nva.emplace_back(nghttp2_nv{
      .name = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(kHeaderNameStatus.data())),
      .value = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(status_str.data())),
      .namelen = kHeaderNameStatus.size(),
      .valuelen = status_str.size(),
      .flags = NGHTTP2_NV_FLAG_NO_COPY_NAME,
  });
  for (const auto& [name, value] : headers) {
    nva.emplace_back(nghttp2_nv{
        .name = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(name.data())),
        .value = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(value.data())),
        .namelen = name.size(),
        .valuelen = value.size(),
        .flags = NGHTTP2_NV_FLAG_NO_COPY_NAME | NGHTTP2_NV_FLAG_NO_COPY_VALUE,
    });
  }

  nghttp2_data_provider content_provider{};
  content_provider.source.ptr = stream.get();
  content_provider.read_callback = [](nghttp2_session* session, int32_t stream_id, uint8_t* buf, size_t length,
                                      uint32_t* data_flags, nghttp2_data_source* source, void* user_data) -> ssize_t {
    const auto* stream = static_cast<Stream*>(source->ptr);
    auto& body = stream->GetResponse()->GetMutableBody();
    auto readable_size = body.GetReadableSize();
    if (readable_size == 0) {
      *data_flags |= NGHTTP2_DATA_FLAG_EOF;

      const auto& trailers = stream->GetResponse()->GetTrailers();
      if (!trailers.empty()) {
        *data_flags |= NGHTTP2_DATA_FLAG_NO_END_STREAM;
        auto* server_session = static_cast<ServerSession*>(user_data);
        auto submit_ok = server_session->SubmitTrailer(stream_id, trailers);
        if (submit_ok != 0) {
          AIMRT_ERROR("nghttp2_submit_trailer failed: {}, {}", submit_ok, nghttp2_strerror(submit_ok));
          return NGHTTP2_ERR_CALLBACK_FAILURE;
        }
      }

      return 0;
    }

    size_t bytes_to_read = std::min(length, readable_size);
    body.Read(buf, bytes_to_read);
    body.Consume(bytes_to_read);
    return static_cast<ssize_t>(bytes_to_read);
  };

  auto submit_ok = nghttp2_submit_response(session_, stream_id, nva.data(), nva.size(), &content_provider);
  if (submit_ok != 0) {
    AIMRT_ERROR("nghttp2_submit_response failed: {}", submit_ok);
    return -1;
  }

  return 0;
}

int ServerSession::ParseRecvMessage(std::string_view in) {
  if (!good_client_magic_) [[unlikely]] {
    return HandleClientMagic(in);
  }

  return Session::ParseRecvMessage(in);
}

int ServerSession::SubmitSettings(Http2Settings settings) {
  std::vector<nghttp2_settings_entry> entries{
      {NGHTTP2_SETTINGS_MAX_CONCURRENT_STREAMS, settings.max_concurrent_streams},
      {NGHTTP2_SETTINGS_INITIAL_WINDOW_SIZE, settings.initial_window_size},
  };

  auto submit_ok = nghttp2_submit_settings(session_, NGHTTP2_FLAG_NONE, entries.data(), entries.size());
  if (submit_ok != 0) {
    AIMRT_ERROR("nghttp2_submit_settings failed: {}", submit_ok);
    return -1;
  }

  auto set_ok = nghttp2_session_set_local_window_size(session_, NGHTTP2_FLAG_NONE, 0,
                                                      static_cast<int32_t>(settings.initial_window_size));
  if (set_ok != 0) {
    AIMRT_ERROR("nghttp2_session_set_local_window_size failed: {}", set_ok);
    return -1;
  }

  return 0;
}

int ServerSession::HandleClientMagic(std::string_view in) {
  if (in.empty()) {
    return -1;
  }

  if (in.size() < NGHTTP2_CLIENT_MAGIC_LEN) {
    return 0;
  }

  if (!in.starts_with(NGHTTP2_CLIENT_MAGIC)) {
    return -1;
  }

  auto parse_ok = Session::ParseRecvMessage(in);
  if (parse_ok != 0) {
    return -1;
  }

  good_client_magic_ = true;
  return 0;
}

void ServerSession::OnFullRequest(const RequestPtr& request_ptr) {
  full_request_list_.push_front(request_ptr);
}

std::forward_list<RequestPtr> ServerSession::GetFullRequestList() {
  return std::move(full_request_list_);
}

// Nghttp2 callbacks which will be called by nghttp2_session_mem_recv
namespace {

int OnBeginHeadersCallback(nghttp2_session* session, const nghttp2_frame* frame, void* user_data) {
  AIMRT_TRACE("stream begin headers, stream_id: {}", frame->hd.stream_id);
  if (frame->hd.type != NGHTTP2_HEADERS || frame->headers.cat != NGHTTP2_HCAT_REQUEST) {
    return 0;
  }

  auto* server_session = static_cast<ServerSession*>(user_data);
  auto request = std::make_shared<Request>();
  request->SetStreamId(frame->hd.stream_id);
  auto stream = server_session->CreateStream();
  stream->SetStreamId(frame->hd.stream_id);
  stream->SetSession(server_session);
  stream->SetRequest(request);
  server_session->AddStream(std::move(stream));
  return 0;
}

int OnHeaderCallback(nghttp2_session* session, const nghttp2_frame* frame, const uint8_t* name, size_t name_len,
                     const uint8_t* value, size_t value_len, uint8_t flags, void* user_data) {
  AIMRT_TRACE("stream on header, stream id: {}, name: {}, value: {}", frame->hd.stream_id,
              std::string_view(reinterpret_cast<const char*>(name), name_len),
              std::string_view(reinterpret_cast<const char*>(value), value_len));
  if (frame->hd.type != NGHTTP2_HEADERS || frame->headers.cat != NGHTTP2_HCAT_REQUEST) {
    return 0;
  }

  auto* server_session = static_cast<ServerSession*>(user_data);
  auto stream = server_session->FindStream(frame->hd.stream_id);
  if (!stream) {
    return 0;
  }

  const auto& request = stream->GetRequest();
  auto& url = request->GetMutableUrl();

  std::string_view name_view(reinterpret_cast<const char*>(name), name_len);
  std::string_view value_view(reinterpret_cast<const char*>(value), value_len);
  if (name_len == kHeaderNameMethod.size() && name_view == kHeaderNameMethod) {
    request->SetMethod(value_view);
  } else if (name_len == kHeaderNamePath.size() && name_view == kHeaderNamePath) {
    url.path = value_view;
  } else if (name_len == kHeaderNameScheme.size() && name_view == kHeaderNameScheme) {
    url.protocol = value_view;
  } else if (name_len == kHeaderNameHost.size() && name_view == kHeaderNameHost) {
    url.host = value_view;
  } else if (name_len == kHeaderNameAuthority.size() && name_view == kHeaderNameAuthority) {
    if (url.host.empty()) {
      url.host = value_view;
    }
  } else {
    request->AddHeader(name_view, value_view);
  }

  return 0;
}

int OnFrameRecvCallback(nghttp2_session* session, const nghttp2_frame* frame, void* user_data) {
  AIMRT_TRACE("stream recv frame, stream id: {}", frame->hd.stream_id);
  auto* server_session = static_cast<ServerSession*>(user_data);
  auto stream = server_session->FindStream(frame->hd.stream_id);
  if (!stream) {
    return 0;
  }

  if (frame->hd.flags & NGHTTP2_FLAG_END_STREAM) {
    server_session->OnFullRequest(stream->GetRequest());
    AIMRT_TRACE("stream recv frame, stream id: {}, OnFullRequest", frame->hd.stream_id);
    return 0;
  }

  auto request = stream->GetRequest();
  switch (frame->hd.type) {
    case NGHTTP2_DATA:
      if (frame->hd.flags & NGHTTP2_FLAG_END_STREAM) {
        server_session->OnFullRequest(request);
        AIMRT_TRACE("stream recv frame, stream id: {}, OnFullRequest", frame->hd.stream_id);
      }
      break;
    case NGHTTP2_HEADERS:
      if (frame->headers.cat != NGHTTP2_HCAT_REQUEST) {
        AIMRT_TRACE("stream recv frame, stream id: {}, NGHTTP2_HEADERS, but not request", frame->hd.stream_id);
        break;
      }
      if (frame->hd.flags & NGHTTP2_FLAG_END_STREAM) {
        server_session->OnFullRequest(request);
        AIMRT_TRACE("stream recv frame, stream id: {}, OnFullRequest", frame->hd.stream_id);
        break;
        case NGHTTP2_RST_STREAM:
          // call OnRst
          AIMRT_TRACE("stream recv frame, stream id: {}, OnRst", frame->hd.stream_id);
          break;
        default:
          AIMRT_TRACE("stream recv frame, stream id: {}, default", frame->hd.stream_id);
          break;
      }
  }

  return 0;
}

int OnDataChunkRecvCallback(nghttp2_session* session, uint8_t flags, int32_t stream_id, const uint8_t* data, size_t len,
                            void* user_data) {
  AIMRT_TRACE("stream recv data chunk, stream id: {}, chunk size: {}", stream_id, len);
  auto* server_session = static_cast<ServerSession*>(user_data);
  auto stream = server_session->FindStream(stream_id);
  if (!stream) {
    return 0;
  }

  auto request = stream->GetRequest();
  request->Write(data, len);

  return 0;
}

int OnStreamCloseCallback(nghttp2_session* session, int32_t stream_id, uint32_t error_code, void* user_data) {
  AIMRT_TRACE("stream close, stream id: {}", stream_id);
  auto* server_session = static_cast<ServerSession*>(user_data);
  server_session->RemoveStream(stream_id);
  return 0;
}

int OnFrameNotSendCallback(nghttp2_session* session, const nghttp2_frame* frame, int32_t error_code, void* user_data) {
  AIMRT_ERROR("stream not send frame, stream id: {}, frame type: {}, error: {}, {}",
              frame->hd.stream_id, frame->hd.type, error_code, nghttp2_strerror(error_code));
  return 0;
}

}  // namespace
// End of source code from trpc-cpp.

}  // namespace aimrt::plugins::grpc_plugin::http2