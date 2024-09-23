// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <forward_list>

#include "grpc_plugin/http2/session.h"

namespace aimrt::plugins::grpc_plugin::http2 {
// The following source code is from trpc-cpp.
// Copied and modified from
// https://github.com/trpc-group/trpc-cpp/blob/v1.2.0/trpc/codec/grpc/http2/server_session.h

class ServerSession : public Session {
 public:
  int InitSession(Http2Settings settings) override;

  int SubmitResponse(const ResponsePtr& response_ptr) override;

  int ParseRecvMessage(std::string_view in) override;

  void OnFullRequest(const RequestPtr& request_ptr);
  [[nodiscard]] std::forward_list<RequestPtr> GetFullRequestList();

 private:
  int SubmitSettings(Http2Settings settings);

  int HandleClientMagic(std::string_view in);

 private:
  bool good_client_magic_ = false;

  std::forward_list<RequestPtr> full_request_list_;
};
// End of source code from trpc-cpp.

}  // namespace aimrt::plugins::grpc_plugin::http2