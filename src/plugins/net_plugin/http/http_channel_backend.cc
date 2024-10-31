// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "net_plugin/http/http_channel_backend.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "aimrt_module_cpp_interface/util/type_support.h"

#include "net_plugin/global.h"
#include "util/url_encode.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::net_plugin::HttpChannelBackend::Options> {
  using Options = aimrt::plugins::net_plugin::HttpChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["pub_topics_options"] = YAML::Node();
    for (const auto& pub_topic_options : rhs.pub_topics_options) {
      Node pub_topic_options_node;
      pub_topic_options_node["topic_name"] = pub_topic_options.topic_name;
      pub_topic_options_node["server_url_list"] = pub_topic_options.server_url_list;
      node["pub_topics_options"].push_back(pub_topic_options_node);
    }

    node["sub_topics_options"] = YAML::Node();
    for (const auto& sub_topic_options : rhs.sub_topics_options) {
      Node sub_topic_options_node;
      sub_topic_options_node["topic_name"] = sub_topic_options.topic_name;
      node["sub_topics_options"].push_back(sub_topic_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["pub_topics_options"] && node["pub_topics_options"].IsSequence()) {
      for (const auto& pub_topic_options_node : node["pub_topics_options"]) {
        auto pub_topic_options = Options::PubTopicOptions{
            .topic_name = pub_topic_options_node["topic_name"].as<std::string>(),
            .server_url_list = pub_topic_options_node["server_url_list"].as<std::vector<std::string>>()};

        rhs.pub_topics_options.emplace_back(std::move(pub_topic_options));
      }
    }

    if (node["sub_topics_options"] && node["sub_topics_options"].IsSequence()) {
      for (const auto& sub_topic_options_node : node["sub_topics_options"]) {
        auto sub_topic_options = Options::SubTopicOptions{
            .topic_name = sub_topic_options_node["topic_name"].as<std::string>()};

        rhs.sub_topics_options.emplace_back(std::move(sub_topic_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::net_plugin {

// void HttpChannelBackend::Initialize(YAML::Node options_node) {
//   AIMRT_CHECK_ERROR_THROW(
//       std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
//       "Http channel backend can only be initialized once.");

//   if (options_node && !options_node.IsNull())
//     options_ = options_node.as<Options>();

//   options_node = options_;
// }
void HttpChannelBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Http channel backend can only be initialized once.");

  if (options_node && !options_node.IsNull()) {
    options_ = options_node.as<Options>();
  }

  if (options_node["pub_topics_options"] && options_node["pub_topics_options"].IsSequence()) {
    for (const auto& pub_topic_options_node : options_node["pub_topics_options"]) {
      if (pub_topic_options_node["host_header"]) {
        host_header_ = pub_topic_options_node["host_header"].as<std::string>();  // Load host header from configuration
      }
    }
  }
}

void HttpChannelBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void HttpChannelBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  http_svr_ptr_->Shutdown();
  http_cli_pool_ptr_->Shutdown();
}

bool HttpChannelBackend::RegisterPublishType(
    const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    namespace util = aimrt::common::util;

    const auto& info = publish_type_wrapper.info;

    std::vector<std::string> server_url_list;

    auto find_option_itr = std::find_if(
        options_.pub_topics_options.begin(), options_.pub_topics_options.end(),
        [topic_name = info.topic_name](const Options::PubTopicOptions& pub_option) {
          try {
            return std::regex_match(topic_name.begin(), topic_name.end(), std::regex(pub_option.topic_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       pub_option.topic_name, topic_name, e.what());
            return false;
          }
        });

    if (find_option_itr != options_.pub_topics_options.end()) {
      server_url_list = find_option_itr->server_url_list;
    }

    std::vector<aimrt::common::util::Url<std::string>> server_url_st_vec;
    for (const auto& publish_add : server_url_list) {
      auto url_op = util::ParseUrl<std::string>(publish_add);
      if (url_op) {
        server_url_st_vec.emplace_back(*url_op);
      } else {
        AIMRT_WARN("Can not parse url: {}, topic: {}", publish_add, info.topic_name);
      }
    }

    pub_cfg_info_map_.emplace(
        info.topic_name,
        PubCfgInfo{
            .server_url_st_vec = std::move(server_url_st_vec)});

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool HttpChannelBackend::Subscribe(
    const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    namespace asio = boost::asio;
    namespace http = boost::beast::http;
    namespace util = aimrt::common::util;

    const auto& info = subscribe_wrapper.info;

    std::string pattern = std::string("/channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type);

    auto find_itr = http_subscribe_wrapper_map_.find(pattern);
    if (find_itr != http_subscribe_wrapper_map_.end()) {
      find_itr->second->AddSubscribeWrapper(&subscribe_wrapper);
      return true;
    }

    auto sub_tool_unique_ptr = std::make_unique<aimrt::runtime::core::channel::SubscribeTool>();
    sub_tool_unique_ptr->AddSubscribeWrapper(&subscribe_wrapper);

    auto* sub_tool_ptr = sub_tool_unique_ptr.get();

    http_subscribe_wrapper_map_.emplace(pattern, std::move(sub_tool_unique_ptr));

    aimrt::common::net::AsioHttpServer::HttpHandle<http::dynamic_body> http_handle =
        [this, topic_name = info.topic_name, sub_tool_ptr](
            const http::request<http::dynamic_body>& req,
            http::response<http::dynamic_body>& rsp,
            std::chrono::nanoseconds timeout)
        -> asio::awaitable<aimrt::common::net::AsioHttpServer::HttpHandleStatus> {
      // 获取序列化类型
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

      rsp.keep_alive(req.keep_alive());
      rsp.prepare_payload();

      // context
      auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

      ctx_ptr->SetSerializationType(serialization_type);

      // 从http header中读取其他字段到context中
      for (auto const& field : req) {
        ctx_ptr->SetMetaValue(
            aimrt::common::util::HttpHeaderDecode(field.name_string()),
            aimrt::common::util::HttpHeaderDecode(field.value()));
      }

      ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

      // 获取消息buf
      const auto& req_beast_buf = req.body().data();
      std::vector<aimrt_buffer_view_t> buffer_view_vec;

      for (auto const buf : boost::beast::buffers_range_ref(req_beast_buf)) {
        buffer_view_vec.emplace_back(aimrt_buffer_view_t{
            .data = const_cast<void*>(buf.data()),
            .len = buf.size()});
      }

      aimrt::util::BufferArrayView buffer_array_view(buffer_view_vec);

      sub_tool_ptr->DoSubscribeCallback(ctx_ptr, serialization_type, buffer_array_view);

      co_return aimrt::common::net::AsioHttpServer::HttpHandleStatus::kOk;
    };

    http_svr_ptr_->RegisterHttpHandleFunc<http::dynamic_body>(
        pattern, std::move(http_handle));

    AIMRT_INFO("Register http handle for channel, uri '{}'", pattern);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void HttpChannelBackend::Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kStart,
                            "Method can only be called when state is 'Start'.");

    namespace asio = boost::asio;
    namespace http = boost::beast::http;
    namespace util = aimrt::common::util;

    const auto& info = msg_wrapper.info;

    auto find_itr = pub_cfg_info_map_.find(info.topic_name);
    AIMRT_CHECK_ERROR_THROW(
        find_itr != pub_cfg_info_map_.end() && !(find_itr->second.server_url_st_vec.empty()),
        "Server url list is empty for topic '{}'", info.topic_name);

    const auto& server_url_st_vec = find_itr->second.server_url_st_vec;

    // 确定path
    std::string pattern = std::string("/channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type);

    // http req
    auto req_ptr = std::make_shared<http::request<http::dynamic_body>>(
        http::verb::post, pattern, 11);
    req_ptr->set(http::field::user_agent, "aimrt");

    // 确定数据序列化类型，先找ctx，ctx中未配置则找支持的第一种序列化类型
    auto publish_type_support_ref = info.msg_type_support_ref;

    std::string_view serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    if (serialization_type.empty()) {
      serialization_type = publish_type_support_ref.DefaultSerializationType();
    }

    if (serialization_type == "json") {
      req_ptr->set(http::field::content_type, "application/json");
    } else if (serialization_type == "pb") {
      req_ptr->set(http::field::content_type, "application/protobuf");
    } else if (serialization_type == "ros2") {
      req_ptr->set(http::field::content_type, "application/ros2");
    } else {
      req_ptr->set(http::field::content_type, "application/" + std::string(serialization_type));
    }

    // 向http header中设置其他context meta字段
    std::vector<std::string_view> meta_keys = msg_wrapper.ctx_ref.GetMetaKeys();
    for (const auto& item : meta_keys) {
      req_ptr->set(
          aimrt::common::util::HttpHeaderEncode(item),
          aimrt::common::util::HttpHeaderEncode(msg_wrapper.ctx_ref.GetMetaValue(item)));
    }

    // msg序列化
    auto buffer_array_view_ptr = aimrt::runtime::core::channel::SerializeMsgWithCache(msg_wrapper, serialization_type);
    AIMRT_CHECK_ERROR_THROW(
        buffer_array_view_ptr,
        "Msg serialization failed, serialization_type {}, pkg_path: {}, module_name: {}, topic_name: {}, msg_type: {}",
        serialization_type, info.pkg_path, info.module_name, info.topic_name, info.msg_type);

    // 填http req包，直接复制过去
    size_t msg_size = buffer_array_view_ptr->BufferSize();
    auto req_beast_buf = req_ptr->body().prepare(msg_size);

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
    req_ptr->body().commit(msg_size);

    req_ptr->keep_alive(true);

    for (const auto& server_url : server_url_st_vec) {
      asio::co_spawn(
          *io_ptr_,
          [this, server_url, req_ptr]() -> asio::awaitable<void> {
            aimrt::common::net::AsioHttpClient::Options cli_options{
                .host = server_url.host,
                .service = server_url.service};

            auto client_ptr = co_await http_cli_pool_ptr_->GetClient(cli_options);
            if (!client_ptr) [[unlikely]] {
              AIMRT_WARN("Can not get http client!");
              co_return;
            }

            // todo:
            // 解决多地址发送时req设置host时的线程安全问题，除最后一个直接用指针，前几个都用值拷贝
            // host以及其他header字段使用配置进行设置，不要写死

            // Add a lock guard to ensure thread safety when modifying req_ptr
            // {
            //   std::lock_guard<std::mutex> lock(req_mutex_);
            //   req_ptr->set(http::field::host, server_url.host);
            //   req_ptr->prepare_payload();
            // }

            // Use the configured host header to avoid hardcoding
            req.set(http::field::host, host_header);
            req.prepare_payload();

            auto rsp = co_await client_ptr->HttpSendRecvCo<http::dynamic_body, http::dynamic_body>(*req_ptr);

            if (rsp.result() != http::status::ok) {
              AIMRT_WARN("http channel publish get error: {} {}",
                         rsp.result_int(),
                         std::string(rsp.reason().data(), rsp.reason().size()));
            }
          },
          asio::detached);
    }

    return;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

}  // namespace aimrt::plugins::net_plugin
