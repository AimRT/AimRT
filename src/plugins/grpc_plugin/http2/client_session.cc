// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "grpc_plugin/http2/client_session.h"

#include <nghttp2/nghttp2.h>

#include "grpc_plugin/global.h"  // IWYU pragma: keep
#include "grpc_plugin/http2/http2.h"
#include "grpc_plugin/http2/request.h"
#include "grpc_plugin/http2/response.h"
#include "util/deferred.h"

namespace aimrt::plugins::grpc_plugin::http2 {
// The following code is from trpc-cpp.
// Copied and modified from
// https://github.com/trpc-group/trpc-cpp/blob/v1.2.0/trpc/codec/grpc/http2/client_session.cc

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

int ClientSession::InitSession(Http2Settings settings) {
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

  new_ok = nghttp2_session_client_new(&session_, callbacks, this);
  if (new_ok != 0) {
    AIMRT_ERROR("nghttp2_session_client_new failed: {}", new_ok);
    return -1;
  }

  auto submit_ok = SubmitSettings(settings);
  if (submit_ok != 0) {
    return -1;
  }

  return 0;
}

int ClientSession::SubmitSettings(Http2Settings settings) {
  std::vector<nghttp2_settings_entry> entries{
      {NGHTTP2_SETTINGS_MAX_CONCURRENT_STREAMS, settings.max_concurrent_streams},
      {NGHTTP2_SETTINGS_INITIAL_WINDOW_SIZE, settings.initial_window_size},
  };

  auto submit_ok = nghttp2_submit_settings(session_, NGHTTP2_FLAG_NONE, entries.data(), entries.size());
  if (submit_ok != 0) {
    AIMRT_ERROR("nghttp2_submit_settings failed: {}", submit_ok);
    return -1;
  }

  return 0;
}

int ClientSession::SubmitRequest(const RequestPtr& request_ptr) {
  std::vector<nghttp2_nv> nva;
  nva.reserve(request_ptr->GetHeaders().size() + 4);
  const auto& url_struct = request_ptr->GetUrl();
  nva.emplace_back(CreatePairWithoutCopy(kHeaderNameMethod, request_ptr->GetMethod()));
  nva.emplace_back(CreatePairWithoutCopy(kHeaderNamePath, url_struct.path));
  nva.emplace_back(CreatePairWithoutCopy(kHeaderNameScheme, url_struct.protocol));
  nva.emplace_back(CreatePairWithoutCopy(kHeaderNameAuthority, url_struct.host));
  for (const auto& [name, value] : request_ptr->GetHeaders()) {
    nva.emplace_back(CreatePairWithoutCopy(name, value));
  }

  StreamPtr stream_ptr = std::make_shared<Stream>();

  nghttp2_data_provider data_provider;
  data_provider.source.ptr = stream_ptr.get();
  data_provider.read_callback = [](nghttp2_session* session, int32_t stream_id, uint8_t* buf, size_t length,
                                   uint32_t* data_flags, nghttp2_data_source* source, void* user_data) -> ssize_t {
    auto* stream = static_cast<Stream*>(source->ptr);
    auto& body = stream->GetRequest()->GetMutableBody();
    auto readable_size = body.GetReadableSize();
    if (readable_size == 0) {
      *data_flags |= NGHTTP2_DATA_FLAG_EOF;
      return 0;
    }

    size_t bytes_to_read = std::min(length, readable_size);
    body.Read(buf, bytes_to_read);
    body.Consume(bytes_to_read);
    return static_cast<ssize_t>(bytes_to_read);
  };

  auto stream_id = nghttp2_submit_request(session_, nullptr, nva.data(), nva.size(), &data_provider, nullptr);

  AIMRT_TRACE("nghttp2_submit_request, stream_id: {}", stream_id);

  if (stream_id < 0) {
    AIMRT_ERROR("nghttp2_submit_request failed: {}", stream_id);
    return -1;
  }

  request_ptr->SetStreamId(stream_id);

  auto response_ptr = std::make_shared<Response>();
  response_ptr->SetStreamId(stream_id);

  stream_ptr->SetStreamId(stream_id);
  stream_ptr->SetRequest(request_ptr);
  stream_ptr->SetResponse(response_ptr);
  stream_ptr->SetSession(this);

  AddStream(std::move(stream_ptr));
  return 0;
}

void ClientSession::OnFullResponse(const ResponsePtr& response_ptr) {
  full_response_list_.push_front(response_ptr);
}

std::forward_list<ResponsePtr> ClientSession::GetFullResponseList() {
  return std::move(full_response_list_);
}

namespace {

int OnBeginHeadersCallback(nghttp2_session* session, const nghttp2_frame* frame, void* user_data) {
  AIMRT_TRACE("stream begin headers, stream_id: {}", frame->hd.stream_id);
  return 0;
}

int OnHeaderCallback(nghttp2_session* session, const nghttp2_frame* frame, const uint8_t* name, size_t name_len,
                     const uint8_t* value, size_t value_len, uint8_t flags, void* user_data) {
  AIMRT_TRACE("stream on header, stream id: {}, name: {}, value: {}", frame->hd.stream_id,
              std::string_view(reinterpret_cast<const char*>(name), name_len),
              std::string_view(reinterpret_cast<const char*>(value), value_len));
  auto* client_session = static_cast<ClientSession*>(user_data);
  switch (frame->hd.type) {
    case NGHTTP2_HEADERS:
      auto stream = client_session->FindStream(frame->hd.stream_id);
      if (!stream) {
        return 0;
      }
      auto response = stream->GetResponse();
      auto name_view = std::string_view(reinterpret_cast<const char*>(name), name_len);
      auto value_view = std::string_view(reinterpret_cast<const char*>(value), value_len);
      if (name_len == kHeaderNameStatus.size() && name_view == kHeaderNameStatus) {
        response->SetHttpStatus(static_cast<HttpStatus>(std::stoi(std::string(value_view))));
      } else {
        // Attention: This is only suitable for grpc which must send trailers.
        if (frame->hd.flags & NGHTTP2_FLAG_END_STREAM) {
          response->AddTrailer(name_view, value_view);
        } else {
          response->AddHeader(name_view, value_view);
        }
      }
      break;
  }

  return 0;
}

int OnFrameRecvCallback(nghttp2_session* session, const nghttp2_frame* frame, void* user_data) {
  AIMRT_TRACE("stream recv frame, stream id: {}", frame->hd.stream_id);
  auto* client_session = static_cast<ClientSession*>(user_data);
  auto stream = client_session->FindStream(frame->hd.stream_id);
  if (!stream) {
    return 0;
  }

  switch (frame->hd.type) {
    case NGHTTP2_DATA:
      if (frame->hd.flags & NGHTTP2_FLAG_END_STREAM) {
        client_session->OnFullResponse(stream->GetResponse());
        AIMRT_TRACE("stream recv frame, stream id: {}, OnFullResponse", frame->hd.stream_id);
        break;
      }
      break;
    case NGHTTP2_HEADERS:
      if (frame->hd.flags & NGHTTP2_FLAG_END_STREAM) {
        client_session->OnFullResponse(stream->GetResponse());
        AIMRT_TRACE("stream recv frame, stream id: {}, OnFullResponse", frame->hd.stream_id);
        break;
      }
      break;
  }

  return 0;
}

int OnDataChunkRecvCallback(nghttp2_session* session, uint8_t flags, int32_t stream_id, const uint8_t* data, size_t len,
                            void* user_data) {
  AIMRT_TRACE("stream recv data chunk, stream id: {}, len: {}", stream_id, len);
  auto* client_session = static_cast<ClientSession*>(user_data);
  auto stream = client_session->FindStream(stream_id);
  if (!stream) {
    return 0;
  }
  auto response = stream->GetResponse();
  response->Write(data, len);
  return 0;
}

int OnStreamCloseCallback(nghttp2_session* session, int32_t stream_id, uint32_t error_code, void* user_data) {
  AIMRT_TRACE("stream close, stream id: {}, error code: {}", stream_id, error_code);
  auto* client_session = static_cast<ClientSession*>(user_data);
  auto stream = client_session->FindStream(stream_id);
  if (!stream) {
    return 0;
  }
  client_session->RemoveStream(stream_id);
  return 0;
}

int OnFrameNotSendCallback(nghttp2_session* session, const nghttp2_frame* frame, int32_t error_code, void* user_data) {
  AIMRT_TRACE("stream not send frame, stream id: {}, error code: {}", frame->hd.stream_id, error_code);
  return 0;
}

}  // namespace
// End of source code from trpc-cpp.

}  // namespace aimrt::plugins::grpc_plugin::http2
