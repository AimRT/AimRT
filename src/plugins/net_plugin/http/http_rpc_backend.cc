// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "net_plugin/http/http_rpc_backend.h"

#include <regex>

#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "aimrt_module_cpp_interface/util/buffer.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/rpc/rpc_backend_tools.h"
#include "net_plugin/global.h"
#include "util/url_encode.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::net_plugin::HttpRpcBackend::Options> {
  using Options = aimrt::plugins::net_plugin::HttpRpcBackend::Options;

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

namespace aimrt::plugins::net_plugin {

void HttpRpcBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Http Rpc backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  options_node = options_;
}

void HttpRpcBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void HttpRpcBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  http_svr_ptr_->Shutdown();
  http_cli_pool_ptr_->Shutdown();
}

bool HttpRpcBackend::RegisterServiceFunc(
    const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Service func can only be registered when state is 'Init'.");
      return false;
    }

    namespace asio = boost::asio;
    namespace http = boost::beast::http;

    std::string pattern =
        std::string("/rpc") + std::string(GetRealFuncName(service_func_wrapper.info.func_name));

    aimrt::common::net::AsioHttpServer::HttpHandle<http::dynamic_body> http_handle =
        [this, &service_func_wrapper](
            const http::request<http::dynamic_body>& req,
            http::response<http::dynamic_body>& rsp,
            std::chrono::nanoseconds timeout)
        -> asio::awaitable<aimrt::common::net::AsioHttpServer::HttpHandleStatus> {
      // 创建 service invoke wrapper
      auto service_invoke_wrapper_ptr = std::make_shared<runtime::core::rpc::InvokeWrapper>(
          runtime::core::rpc::InvokeWrapper{.info = service_func_wrapper.info});
      const auto& info = service_invoke_wrapper_ptr->info;

      // 创建 service ctx
      auto ctx_ptr = std::make_shared<aimrt::rpc::Context>(aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT);
      service_invoke_wrapper_ptr->ctx_ref = ctx_ptr;

      // 序列化类型
      std::string serialization_type;
      auto req_content_type_itr = req.find(http::field::content_type);
      AIMRT_CHECK_ERROR_THROW(req_content_type_itr != req.end(),
                              "Http req has no content type.");

      auto req_content_type_boost_sw = req_content_type_itr->value();
      std::string_view req_content_type(req_content_type_boost_sw.data(), req_content_type_boost_sw.size());
      if (req_content_type == "application/json" ||
          req_content_type == "application/json charset=utf-8") {
        serialization_type = "json";
        rsp.set(http::field::content_type, "application/json");
      } else if (req_content_type == "application/protobuf") {
        serialization_type = "pb";
        rsp.set(http::field::content_type, "application/protobuf");
      } else if (req_content_type == "application/ros2") {
        serialization_type = "ros2";
        rsp.set(http::field::content_type, "application/ros2");
      } else {
        AIMRT_ERROR_THROW("Http req has invalid content type {}.", req_content_type);
      }

      ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE, serialization_type);

      // 从http header中读取其他字段到context中
      for (auto const& field : req) {
        ctx_ptr->SetMetaValue(
            aimrt::common::util::HttpHeaderDecode(field.name_string()),
            aimrt::common::util::HttpHeaderDecode(field.value()));
      }

      ctx_ptr->SetFunctionName(info.func_name);
      ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND, Name());

      ctx_ptr->SetTimeout(timeout);

      // service req反序列化
      const auto& req_beast_buf = req.body().data();
      std::vector<aimrt_buffer_view_t> buffer_view_vec;

      for (auto const buf : boost::beast::buffers_range_ref(req_beast_buf)) {
        buffer_view_vec.emplace_back(aimrt_buffer_view_t{
            .data = const_cast<void*>(buf.data()),
            .len = buf.size()});
      }

      aimrt_buffer_array_view_t buffer_array_view{
          .data = buffer_view_vec.data(),
          .len = buffer_view_vec.size()};

      std::shared_ptr<void> service_req_ptr = info.req_type_support_ref.CreateSharedPtr();
      service_invoke_wrapper_ptr->req_ptr = service_req_ptr.get();

      bool deserialize_ret = info.req_type_support_ref.Deserialize(
          serialization_type, buffer_array_view, service_req_ptr.get());

      AIMRT_CHECK_ERROR_THROW(deserialize_ret, "Http req deserialize failed.");

      // service rsp创建
      std::shared_ptr<void> service_rsp_ptr = info.rsp_type_support_ref.CreateSharedPtr();
      service_invoke_wrapper_ptr->rsp_ptr = service_rsp_ptr.get();

      // 设置回调
      uint32_t ret_code = 0;
      auto sig_timer_ptr = std::make_shared<asio::steady_timer>(*io_ptr_, std::chrono::nanoseconds::max());

      service_invoke_wrapper_ptr->callback =
          [service_invoke_wrapper_ptr,
           ctx_ptr,
           &req,
           &rsp,
           service_req_ptr,
           service_rsp_ptr,
           serialization_type{std::move(serialization_type)},
           sig_timer_ptr,
           &ret_code](aimrt::rpc::Status status) {
            if (!status.OK()) [[unlikely]] {
              ret_code = status.Code();

            } else {
              // service rsp序列化
              auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeRspWithCache(*service_invoke_wrapper_ptr, serialization_type);

              if (!buffer_array_view_ptr) [[unlikely]] {
                ret_code = AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED;
              } else {
                // 填http rsp包，直接复制过去
                size_t rsp_size = buffer_array_view_ptr->BufferSize();
                auto rsp_beast_buf = rsp.body().prepare(rsp_size);

                const auto* data = buffer_array_view_ptr->Data();
                auto buffer_array_pos = 0;
                size_t buffer_pos = 0;

                for (auto buf : boost::beast::buffers_range_ref(rsp_beast_buf)) {
                  size_t cur_beast_buf_pos = 0;
                  while (cur_beast_buf_pos < buf.size()) {
                    size_t cur_beast_buffer_size = buf.size() - cur_beast_buf_pos;
                    size_t cur_buffer_size = data[buffer_array_pos].len - buffer_pos;

                    size_t cur_copy_size = std::min(cur_beast_buffer_size, cur_buffer_size);

                    memcpy(static_cast<char*>(buf.data()) + cur_beast_buf_pos,
                           static_cast<const char*>(data[buffer_array_pos].data) + buffer_pos,
                           cur_copy_size);

                    buffer_pos += cur_copy_size;
                    if (buffer_pos == data[buffer_array_pos].len) {
                      ++buffer_array_pos;
                      buffer_pos = 0;
                    }

                    cur_beast_buf_pos += cur_copy_size;
                  }
                }
                rsp.body().commit(rsp_size);

                rsp.keep_alive(req.keep_alive());
                rsp.prepare_payload();
              }
            }
            sig_timer_ptr->expires_at(std::chrono::steady_clock::time_point::min());
          };

      // service rpc调用
      service_func_wrapper.service_func(service_invoke_wrapper_ptr);

      try {
        co_await sig_timer_ptr->async_wait(asio::use_awaitable);
      } catch (const boost::system::system_error& e) {
        if (e.code() != boost::asio::error::operation_aborted) [[unlikely]] {
          AIMRT_ERROR("Http handle for rpc, async_wait get exception: {}", e.what());
          ret_code = aimrt_rpc_status_code_t::AIMRT_RPC_STATUS_SVR_BACKEND_INTERNAL_ERROR;
        }
      }

      AIMRT_CHECK_ERROR_THROW(ret_code == 0, "Handle rpc failed, code: {}.", ret_code);

      co_return aimrt::common::net::AsioHttpServer::HttpHandleStatus::kOk;
    };

    http_svr_ptr_->RegisterHttpHandleFunc<http::dynamic_body>(
        pattern, std::move(http_handle));
    AIMRT_INFO("Register http handle for rpc, uri '{}'", pattern);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool HttpRpcBackend::RegisterClientFunc(
    const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper) noexcept {
  try {
    if (state_.load() != State::kInit) {
      AIMRT_ERROR("Client func can only be registered when state is 'Init'.");
      return false;
    }

    const auto& info = client_func_wrapper.info;

    auto find_client_option = std::find_if(
        options_.clients_options.begin(), options_.clients_options.end(),
        [func_name = GetRealFuncName(info.func_name)](const Options::ClientOptions& client_option) {
          try {
            return std::regex_match(func_name.begin(), func_name.end(), std::regex(client_option.func_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       client_option.func_name, func_name, e.what());
            return false;
          }
        });

    if (find_client_option != options_.clients_options.end()) {
      client_server_url_map_.emplace(GetRealFuncName(info.func_name), find_client_option->server_url);
    }

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void HttpRpcBackend::Invoke(
    const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) noexcept {
  try {
    if (state_.load() != State::kStart) [[unlikely]] {
      AIMRT_WARN("Method can only be called when state is 'Start'.");
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    namespace asio = boost::asio;
    namespace http = boost::beast::http;
    namespace util = aimrt::common::util;

    const auto& info = client_invoke_wrapper_ptr->info;

    auto real_func_name = GetRealFuncName(info.func_name);

    // 检查ctx，to_addr优先级：ctx > server_url
    auto to_addr = client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR);
    if (to_addr.empty()) {
      auto find_itr = client_server_url_map_.find(real_func_name);
      if (find_itr != client_server_url_map_.end()) {
        to_addr = find_itr->second;
      } else {
        AIMRT_WARN("Server url is not set for func: {}", info.func_name);
        client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_ADDR));
        return;
      }
    }

    auto url = util::ParseUrl<std::string_view>(to_addr);
    if (!url) {
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_ADDR));
      return;
    }

    asio::co_spawn(
        *io_ptr_,
        [http_cli_pool_ptr{http_cli_pool_ptr_},
         client_invoke_wrapper_ptr,
         url]() -> asio::awaitable<void> {
          const auto& info = client_invoke_wrapper_ptr->info;

          std::string url_path(url->path);
          if (url_path.empty()) {
            url_path = "/rpc" + std::string(GetRealFuncName(info.func_name));
          }

          try {
            aimrt::common::net::AsioHttpClient::Options cli_options{
                .host = std::string(url->host),
                .service = std::string(url->service)};

            auto client_ptr = co_await http_cli_pool_ptr->GetClient(cli_options);
            if (!client_ptr) [[unlikely]] {
              AIMRT_WARN("Can not get http client!");
              client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
              co_return;
            }

            http::request<http::dynamic_body> req{http::verb::post, url_path, 11};
            req.set(http::field::host, url->host);
            req.set(http::field::user_agent, "aimrt");

            auto timeout = client_invoke_wrapper_ptr->ctx_ref.Timeout();
            if (timeout <= std::chrono::nanoseconds(0))
              timeout = std::chrono::seconds(5);
            req.set(http::field::timeout,
                    std::to_string(std::chrono::duration_cast<std::chrono::seconds>(timeout).count()));

            std::string serialization_type(
                client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE));
            if (serialization_type == "json") {
              req.set(http::field::content_type, "application/json");
            } else if (serialization_type == "pb") {
              req.set(http::field::content_type, "application/protobuf");
            } else if (serialization_type == "ros2") {
              req.set(http::field::content_type, "application/ros2");
            } else {
              client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_INVALID_SERIALIZATION_TYPE));
              co_return;
            }

            // 向http header中设置其他context meta字段
            std::vector<std::string_view> meta_keys = client_invoke_wrapper_ptr->ctx_ref.GetMetaKeys();
            for (const auto& item : meta_keys) {
              req.set(
                  aimrt::common::util::HttpHeaderEncode(item),
                  aimrt::common::util::HttpHeaderEncode(client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(item)));
            }

            // client req序列化
            auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeReqWithCache(*client_invoke_wrapper_ptr, serialization_type);
            if (!buffer_array_view_ptr) [[unlikely]] {
              // 序列化失败
              client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED));
              co_return;
            }

            // 填http req包，直接复制过去
            size_t req_size = buffer_array_view_ptr->BufferSize();
            auto req_beast_buf = req.body().prepare(req_size);

            const auto* data = buffer_array_view_ptr->Data();
            auto buffer_array_pos = 0;
            size_t buffer_pos = 0;

            for (auto buf : boost::beast::buffers_range_ref(req_beast_buf)) {
              size_t cur_beast_buf_pos = 0;
              while (cur_beast_buf_pos < buf.size()) {
                size_t cur_beast_buffer_size = buf.size() - cur_beast_buf_pos;
                size_t cur_buffer_size = data[buffer_array_pos].len - buffer_pos;

                size_t cur_copy_size = std::min(cur_beast_buffer_size, cur_buffer_size);

                memcpy(static_cast<char*>(buf.data()) + cur_beast_buf_pos,
                       static_cast<const char*>(data[buffer_array_pos].data) + buffer_pos,
                       cur_copy_size);

                buffer_pos += cur_copy_size;
                if (buffer_pos == data[buffer_array_pos].len) {
                  ++buffer_array_pos;
                  buffer_pos = 0;
                }

                cur_beast_buf_pos += cur_copy_size;
              }
            }
            req.body().commit(req_size);

            req.prepare_payload();
            req.keep_alive(true);

            auto rsp = co_await client_ptr->HttpSendRecvCo<http::dynamic_body, http::dynamic_body>(req, timeout);

            // 检查rsp header等参数（TODO）

            // client rsp 反序列化
            const auto& rsp_beast_buf = rsp.body().data();
            std::vector<aimrt_buffer_view_t> buffer_view_vec;

            for (auto const buf : boost::beast::buffers_range_ref(rsp_beast_buf)) {
              buffer_view_vec.emplace_back(aimrt_buffer_view_t{
                  .data = buf.data(),
                  .len = buf.size()});
            }

            aimrt_buffer_array_view_t buffer_array_view{
                .data = buffer_view_vec.data(),
                .len = buffer_view_vec.size()};

            bool deserialize_ret = info.rsp_type_support_ref.Deserialize(
                serialization_type, buffer_array_view, client_invoke_wrapper_ptr->rsp_ptr);

            if (!deserialize_ret) {
              // 反序列化失败
              client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED));
              co_return;
            }

            client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_OK));

            co_return;
          } catch (const std::exception& e) {
            AIMRT_WARN("Http call get exception, info: {}", e.what());
          }

          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_UNKNOWN));
          co_return;
        },
        asio::detached);
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

}  // namespace aimrt::plugins::net_plugin