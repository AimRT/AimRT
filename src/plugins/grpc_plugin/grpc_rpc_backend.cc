// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "grpc_plugin/grpc_rpc_backend.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <regex>
#include <unordered_map>
#include <vector>

#include <boost/asio.hpp>

#include "aimrt_module_c_interface/rpc/rpc_context_base.h"
#include "aimrt_module_c_interface/rpc/rpc_status_base.h"
#include "aimrt_module_c_interface/util/buffer_base.h"
#include "aimrt_module_cpp_interface/rpc/rpc_context.h"
#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "core/rpc/rpc_backend_tools.h"
#include "core/rpc/rpc_invoke_wrapper.h"
#include "core/util/version.h"
#include "grpc_plugin/client/options.h"
#include "grpc_plugin/global.h"  // IWYU pragma: keep
#include "grpc_plugin/grpc/message.h"
#include "grpc_plugin/grpc/timeout.h"
#include "grpc_plugin/http2/request.h"
#include "grpc_plugin/http2/response.h"
#include "util/log_util.h"
#include "util/url_parser.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::grpc_plugin::GrpcRpcBackend::Options> {
  using Options = aimrt::plugins::grpc_plugin::GrpcRpcBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["clients_options"] = YAML::Node();
    for (const auto& client_options : rhs.clients_options) {
      Node client_options_node;
      client_options_node["func_name"] = client_options.func_name;
      client_options_node["server_url"] = client_options.server_url;
      node["clients_options"].push_back(client_options_node);
    }

    node["servers_options"] = YAML::Node();
    for (const auto& server_options : rhs.servers_options) {
      Node server_options_node;
      server_options_node["func_name"] = server_options.func_name;
      node["servers_options"].push_back(server_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["clients_options"] && node["clients_options"].IsSequence()) {
      for (const auto& client_options_node : node["clients_options"]) {
        auto client_options = Options::ClientOptions{
            .func_name = client_options_node["func_name"].as<std::string>(),
            .server_url = client_options_node["server_url"].as<std::string>()};

        rhs.clients_options.emplace_back(std::move(client_options));
      }
    }

    if (node["servers_options"] && node["servers_options"].IsSequence()) {
      for (const auto& server_options_node : node["servers_options"]) {
        auto server_options = Options::ServerOptions{
            .func_name = server_options_node["func_name"].as<std::string>()};

        rhs.servers_options.emplace_back(std::move(server_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::grpc_plugin {

namespace asio = boost::asio;
namespace util = aimrt::common::util;
namespace chrono = std::chrono;

using aimrt::runtime::core::util::GetAimRTVersion;

namespace {

void CheckGrpcMessageBody(const http2::SimpleBuffer& buffer) {
  auto body_str_view = buffer.GetStringView();
  std::optional<grpc::GrpcMessagePrefix> prefix = grpc::ParseGrpcMessagePrefix(body_str_view);
  AIMRT_CHECK_ERROR_THROW(prefix,
                          "Http2 request body is not a valid grpc message, body_len: {}", body_str_view.size());
  AIMRT_CHECK_ERROR_THROW(prefix->compression_flag == 0,
                          "Not support grpc compression Now");
  AIMRT_CHECK_ERROR_THROW(body_str_view.size() == grpc::kGrpcMessagePrefixSize + prefix->message_length,
                          "Http2 request body size not equal to grpc message length");
}

void CheckGrpcReqHeaders(const http2::RequestPtr& req) {
  // Check the te, must be trailers
  if (auto te_itr = req->GetHeaders().find("te"); te_itr == req->GetHeaders().end()) {
    AIMRT_ERROR_THROW("te is not set for grpc");
  } else if (te_itr->second != "trailers") {
    AIMRT_ERROR_THROW("te is {}, which is not supported", te_itr->second);
  }

  // Check the method, must be POST
  AIMRT_CHECK_ERROR_THROW(req->GetMethod() == "POST",
                          "Method is {}, which is not supported", req->GetMethod());

  // Check the scheme, only support http now (TODO: support https)
  AIMRT_CHECK_ERROR_THROW(req->GetUrl().protocol == "http",
                          "Scheme is {}, which is not supported", req->GetUrl().protocol);
}

}  // namespace

void GrpcRpcBackend::Initialize(YAML::Node options_node) {
  AIMRT_DEBUG("Initialize grpc rpc backend.");

  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Http Rpc backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  options_node = options_;
}

void GrpcRpcBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void GrpcRpcBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;
}

bool GrpcRpcBackend::RegisterServiceFunc(
    const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Service func can only be registered when state is 'Init'.");
      return false;
    }

    const auto& func_name = service_func_wrapper.info.func_name;

    AIMRT_DEBUG("Register service func: {}", func_name);
    if (!func_name.starts_with("pb:") && !func_name.starts_with("ros2:")) {
      AIMRT_ERROR("Service func name should start with 'pb:' or 'ros2:'.");
      return false;
    }

    // pb:/aimrt.protocols.example.ExampleService/GetBarData -> /aimrt.protocols.example.ExampleService/GetBarData
    // ros2:/example_ros2/srv/RosTestRpc -> /example_ros2/srv/RosTestRpc
    auto pattern = std::string(rpc::GetFuncNameWithoutPrefix(func_name));

    plugins::grpc_plugin::server::HttpHandle http_handle =
        [this, &service_func_wrapper](
            const http2::RequestPtr& req,
            http2::ResponsePtr& rsp)
        -> boost::asio::awaitable<void> {
      AIMRT_TRACE("Http2 handle for rpc, path: {}", req->GetUrl().path);

      CheckGrpcReqHeaders(req);
      CheckGrpcMessageBody(req->GetBody());

      auto service_invoke_wrapper_ptr = std::make_shared<runtime::core::rpc::InvokeWrapper>(
          runtime::core::rpc::InvokeWrapper{.info = service_func_wrapper.info});

      auto ctx_ptr = std::make_shared<rpc::Context>(aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT);
      service_invoke_wrapper_ptr->ctx_ref = ctx_ptr;

      // Set the serialization type
      auto content_type_itr = req->GetHeaders().find("content-type");
      AIMRT_CHECK_ERROR_THROW(content_type_itr != req->GetHeaders().end(), "content-type is not set for grpc");

      static const std::unordered_map<std::string_view, std::string_view> kContentTypeToSerializationTypeMap = {
          {"application/grpc+json", "json"},
          {"application/grpc+json charset=utf-8", "json"},
          {"application/grpc", "pb"},
          {"application/grpc+proto", "pb"},
          {"application/grpc+ros2", "ros2"}};

      auto find_itr = kContentTypeToSerializationTypeMap.find(content_type_itr->second);
      AIMRT_CHECK_ERROR_THROW(find_itr != kContentTypeToSerializationTypeMap.end(),
                              "Unsupported content-type: {}", content_type_itr->second);
      ctx_ptr->SetSerializationType(find_itr->second);

      // Set the metadata
      for (const auto& [key, value] : req->GetHeaders()) {
        AIMRT_DEBUG("Http2 handle for rpc, key: {}, value: {}", key, value);
        ctx_ptr->SetMetaValue(key, value);
      }
      ctx_ptr->SetFunctionName(service_func_wrapper.info.func_name);
      ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND, Name());

      // Parse the timeout
      const auto& req_headers = req->GetHeaders();
      chrono::nanoseconds timeout = grpc::kDefaultTimeout;
      if (auto timeout_itr = req_headers.find("grpc-timeout"); timeout_itr != req_headers.end()) {
        auto timeout_str = timeout_itr->second;
        std::optional<chrono::nanoseconds> timeout_ns = grpc::ParseTimeout(timeout_str);
        if (timeout_ns) {
          timeout = *timeout_ns;
        } else {
          AIMRT_WARN("Invalid grpc-timeout: {}, using default timeout: {}s", timeout_str, timeout.count());
        }
      }
      ctx_ptr->SetTimeout(timeout);

      // Skip the grpc compression flag and length prefix
      auto body_str_view = req->GetBody().GetStringView();
      body_str_view.remove_prefix(grpc::kGrpcMessagePrefixSize);

      // Deserialize the request
      std::vector<aimrt_buffer_view_t> buffer_view_vec;
      buffer_view_vec.push_back({.data = body_str_view.data(),
                                 .len = body_str_view.size()});
      auto buffer_array_view = aimrt_buffer_array_view_t{.data = buffer_view_vec.data(),
                                                         .len = buffer_view_vec.size()};

      auto service_req_ptr = service_func_wrapper.info.req_type_support_ref.CreateSharedPtr();
      service_invoke_wrapper_ptr->req_ptr = service_req_ptr.get();

      bool deserialize_ret = service_func_wrapper.info.req_type_support_ref.Deserialize(
          ctx_ptr->GetSerializationType(), buffer_array_view, service_req_ptr.get());
      AIMRT_CHECK_ERROR_THROW(deserialize_ret, "Http2 request deserialize failed.");
      auto service_rsp_ptr = service_func_wrapper.info.rsp_type_support_ref.CreateSharedPtr();
      service_invoke_wrapper_ptr->rsp_ptr = service_rsp_ptr.get();

      // Set the callback
      uint32_t ret_code = 0;
      auto sig_timer_ptr = std::make_shared<boost::asio::steady_timer>(*io_ptr_, chrono::nanoseconds::max());
      service_invoke_wrapper_ptr->callback =
          [service_invoke_wrapper_ptr,
           serialization_type = ctx_ptr->GetSerializationType(),
           &rsp,
           &ret_code,
           &sig_timer_ptr](aimrt::rpc::Status status) {
            if (!status.OK()) [[unlikely]] {
              ret_code = status.Code();
              sig_timer_ptr->expires_at(chrono::steady_clock::time_point::min());
              return;
            }

            // Serialize the response
            auto buffer_array_view_ptr =
                aimrt::runtime::core::rpc::TrySerializeRspWithCache(*service_invoke_wrapper_ptr, serialization_type);
            if (!buffer_array_view_ptr) [[unlikely]] {
              ret_code = AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED;
              sig_timer_ptr->expires_at(chrono::steady_clock::time_point::min());
              return;
            }

            // Fill the response data
            auto prefix = grpc::EncodeGrpcMessagePrefix(grpc::GrpcMessagePrefix{
                .compression_flag = 0,
                .message_length = static_cast<uint32_t>(buffer_array_view_ptr->BufferSize())});
            rsp->Write(prefix);
            for (size_t i = 0; i < buffer_array_view_ptr->Size(); ++i) {
              auto buffer_array_view = buffer_array_view_ptr->Data()[i];
              rsp->Write(reinterpret_cast<const uint8_t*>(buffer_array_view.data), buffer_array_view.len);
            }

            rsp->AddTrailer("grpc-status", "0");

            sig_timer_ptr->expires_at(chrono::steady_clock::time_point::min());
          };  // service_invoke_wrapper_ptr->callback

      service_func_wrapper.service_func(service_invoke_wrapper_ptr);

      try {
        co_await sig_timer_ptr->async_wait(boost::asio::use_awaitable);
      } catch (const boost::system::system_error& e) {
        if (e.code() != boost::asio::error::operation_aborted) [[unlikely]] {
          AIMRT_ERROR("Http2 handle for rpc, async_wait get exception: {}", e.what());
          ret_code = aimrt_rpc_status_code_t::AIMRT_RPC_STATUS_SVR_BACKEND_INTERNAL_ERROR;
        }
      }

      AIMRT_CHECK_ERROR_THROW(ret_code == 0, "Handle rpc failed, ret_code: {}", ret_code);

      co_return;
    };  // http_handle

    http2_svr_ptr_->RegisterHttpHandleFunc(pattern, std::move(http_handle));
    AIMRT_INFO("Register http2 handle for rpc, uri: {}", pattern);

    return true;
  } catch (std::exception& e) {
    AIMRT_ERROR("Register service func failed, exception: {}", e.what());
    return false;
  }
}

bool GrpcRpcBackend::RegisterClientFunc(const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper) noexcept {
  if (state_.load() != State::kInit) {
    AIMRT_ERROR("Client func can only be registered when state is 'Init'.");
    return false;
  }

  // Basically, we need to find the server url for each client function.
  const auto& info = client_func_wrapper.info;

  auto find_client_option = std::ranges::find_if(
      options_.clients_options,
      [func_name = info.func_name](const Options::ClientOptions& client_option) {
        try {
          auto real_func_name = std::string(rpc::GetFuncNameWithoutPrefix(func_name));
          return std::regex_match(func_name.begin(), func_name.end(),
                                  std::regex(client_option.func_name, std::regex::ECMAScript)) ||
                 std::regex_match(real_func_name.begin(), real_func_name.end(),
                                  std::regex(client_option.func_name, std::regex::ECMAScript));
        } catch (const std::exception& e) {
          AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                     client_option.func_name, func_name, e.what());
          return false;
        }
      });

  if (find_client_option == options_.clients_options.end()) {
    AIMRT_WARN("Server url is not set for func: {}", info.func_name);
  } else {
    // /aimrt.protocols.example.ExampleService/GetBarData -> 127.0.0.1:8080
    client_server_url_map_.emplace(rpc::GetFuncNameWithoutPrefix(info.func_name), find_client_option->server_url);
  }

  return true;
}

void GrpcRpcBackend::Invoke(
    const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) noexcept {
  try {
    if (state_.load() != State::kStart) [[unlikely]] {
      AIMRT_WARN("Method can only be called when state is 'Start'.");
      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    const auto& info = client_invoke_wrapper_ptr->info;
    auto real_func_name = rpc::GetFuncNameWithoutPrefix(info.func_name);

    // check ctx, to_addr priority: ctx > server_url
    auto to_addr = client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR);
    if (to_addr.empty()) {
      auto find_itr = client_server_url_map_.find(real_func_name);
      if (find_itr != client_server_url_map_.end()) {
        to_addr = find_itr->second;
      } else {
        AIMRT_WARN("Server url is not set for func: {}", info.func_name);
        InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_ADDR));
        return;
      }
    }

    auto url = util::ParseUrl<std::string>(to_addr);
    if (!url) {
      InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_ADDR));
      return;
    }
    if (url->path.empty()) {
      url->path = std::string(rpc::GetFuncNameWithoutPrefix(info.func_name));
    }
    AIMRT_TRACE("Http2 cli session send request, remote addr {}, path: {}",
                url->host, url->path);

    asio::co_spawn(
        *io_ptr_,
        [http2_cli_pool_ptr = http2_cli_pool_ptr_,
         client_invoke_wrapper_ptr,
         url]() -> asio::awaitable<void> {
          try {
            client::ClientOptions cli_options{
                .host = std::string(url->host),
                .service = std::string(url->service),
                .http2_settings = http2::Session::Http2Settings{
                    .max_concurrent_streams = 100,
                    .initial_window_size = (1U << 31) - 1,
                }};

            auto client_ptr = co_await http2_cli_pool_ptr->GetClient(cli_options);
            if (!client_ptr) [[unlikely]] {
              AIMRT_WARN("Can not get http client!");
              InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
              co_return;
            }

            auto req_ptr = std::make_shared<http2::Request>();
            req_ptr->SetUrl(*url);
            req_ptr->SetMethod("POST");
            req_ptr->AddHeader("te", "trailers");
            req_ptr->AddHeader("user-agent", std::string("grpc-c++-aimrt/") + GetAimRTVersion());

            auto timeout = client_invoke_wrapper_ptr->ctx_ref.Timeout();
            if (timeout <= chrono::nanoseconds(0)) [[unlikely]] {
              timeout = chrono::nanoseconds::max();
            }
            req_ptr->AddHeader("grpc-timeout", grpc::FormatTimeout(timeout));

            auto [meta_key_vals_array, meta_key_vals_array_len] = client_invoke_wrapper_ptr->ctx_ref.GetMetaKeyValsArray();
            for (size_t ii = 0; ii < meta_key_vals_array_len; ii += 2) {
              auto key = aimrt::util::ToStdStringView(meta_key_vals_array[ii]);
              auto val = aimrt::util::ToStdStringView(meta_key_vals_array[ii + 1]);
              req_ptr->AddHeader(key, val);
            }

            std::string serialization_type(client_invoke_wrapper_ptr->ctx_ref.GetSerializationType());

            static const std::unordered_map<std::string_view, std::string_view> kSerializationTypeToContentTypeMap = {
                {"json", "application/grpc+json"},
                {"pb", "application/grpc+proto"},
                {"ros2", "application/grpc+ros2"}};

            auto find_itr = kSerializationTypeToContentTypeMap.find(serialization_type);
            if (find_itr == kSerializationTypeToContentTypeMap.end()) {
              AIMRT_ERROR("Unsupported serialization type: {}", serialization_type);
              InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_SERIALIZATION_TYPE));
              co_return;
            }
            req_ptr->AddHeader("content-type", find_itr->second);

            auto buffer_array_view_ptr =
                aimrt::runtime::core::rpc::TrySerializeReqWithCache(*client_invoke_wrapper_ptr, serialization_type);
            if (!buffer_array_view_ptr) [[unlikely]] {
              AIMRT_WARN("Serialize request failed.");
              InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED));
              co_return;
            }

            auto prefix = grpc::EncodeGrpcMessagePrefix(grpc::GrpcMessagePrefix{
                .compression_flag = 0,
                .message_length = static_cast<uint32_t>(buffer_array_view_ptr->BufferSize())});
            req_ptr->Write(prefix);

            for (size_t i = 0; i < buffer_array_view_ptr->Size(); ++i) {
              auto buffer_array_view = buffer_array_view_ptr->Data()[i];
              req_ptr->Write(reinterpret_cast<const uint8_t*>(buffer_array_view.data), buffer_array_view.len);
            }

            auto rsp_ptr = co_await client_ptr->HttpSendRecvCo(req_ptr, timeout);

            // Check the grpc status
            auto trailers = rsp_ptr->GetTrailers();
            auto grpc_status_itr = trailers.find("grpc-status");
            if (grpc_status_itr == trailers.end() || grpc_status_itr->second != "0") {
              AIMRT_WARN("Grpc status is not set or non-zero for response.");
              for (const auto& [key, value] : trailers) {
                AIMRT_WARN("Trailer: {} = {}", key, value);
              }
              // TODO(zhangyi): Classify the grpc status code into relevant aimrt_rpc_status_code_t
              // sa: https://grpc.io/docs/guides/status-codes/
              InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_UNKNOWN));
              co_return;
            }

            // Check the grpc message prefix
            auto body_str_view = rsp_ptr->GetBody().GetStringView();
            std::optional<grpc::GrpcMessagePrefix> rsp_prefix = grpc::ParseGrpcMessagePrefix(body_str_view);
            if (!rsp_prefix) [[unlikely]] {
              AIMRT_WARN("Http2 response body is not a valid grpc message, body_len: {}", body_str_view.size());
              InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
              co_return;
            }
            if (rsp_prefix->compression_flag != 0) {
              AIMRT_WARN("Not support grpc compression now");
              InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
              co_return;
            }
            if (body_str_view.size() != 5 + rsp_prefix->message_length) [[unlikely]] {
              AIMRT_WARN("Http2 response body size not equal to grpc message length, body_len: {}, message_len: {}",
                         body_str_view.size(), rsp_prefix->message_length);
              InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
              co_return;
            }

            // Skip the grpc compression flag and length prefix
            body_str_view.remove_prefix(5);

            // Deserialize the response
            std::vector<aimrt_buffer_view_t> buffer_view_vec;
            buffer_view_vec.push_back({.data = body_str_view.data(),
                                       .len = body_str_view.size()});
            auto buffer_array_view = aimrt_buffer_array_view_t{.data = buffer_view_vec.data(),
                                                               .len = buffer_view_vec.size()};
            auto deserialize_ret = client_invoke_wrapper_ptr->info.rsp_type_support_ref.Deserialize(
                serialization_type, buffer_array_view, client_invoke_wrapper_ptr->rsp_ptr);
            if (!deserialize_ret) [[unlikely]] {
              AIMRT_WARN("Deserialize response failed.");
              InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED));
              co_return;
            }

            InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_OK));

          } catch (std::exception& e) {
            AIMRT_ERROR("Invoke failed, exception: {}", e.what());
            InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
          }

          co_return;
        },
        boost::asio::detached);
  } catch (std::exception& e) {
    AIMRT_ERROR("Invoke failed, exception: {}", e.what());
    InvokeCallBack(*client_invoke_wrapper_ptr, aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_UNKNOWN));
  }
}

}  // namespace aimrt::plugins::grpc_plugin