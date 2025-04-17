// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "iceoryx_plugin/iceoryx_channel_backend.h"
#include "aimrt_module_cpp_interface/util/buffer_array_allocator.h"
#include "iceoryx_plugin/global.h"
#include "util/buffer_util.h"
#include "util/url_encode.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::iceoryx_plugin::IceoryxChannelBackend::Options> {
  using Options = aimrt::plugins::iceoryx_plugin::IceoryxChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["sub_default_executor"] = rhs.sub_default_executor;

    node["sub_topics_options"] = YAML::Node();
    for (const auto& sub_topic_options : rhs.sub_topics_options) {
      Node sub_topic_options_node;
      sub_topic_options_node["topic_name"] = sub_topic_options.topic_name;
      sub_topic_options_node["executor"] = sub_topic_options.executor;
      node["sub_topics_options"].push_back(sub_topic_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["sub_default_executor"]) {
      rhs.sub_default_executor = node["sub_default_executor"].as<std::string>();
    }

    if (node["sub_topics_options"] && node["sub_topics_options"].IsSequence()) {
      for (const auto& sub_topic_options_node : node["sub_topics_options"]) {
        auto sub_topic_options = Options::SubTopicOptions{
            .topic_name = sub_topic_options_node["topic_name"].as<std::string>(),
            .executor = sub_topic_options_node["executor"].as<std::string>()};

        rhs.sub_topics_options.emplace_back(std::move(sub_topic_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::iceoryx_plugin {
void IceoryxChannelBackend::Initialize(YAML::Node options_node) {
  // todo: check options_node->shm_init_size

  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Iceoryx channel backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  options_node = options_;
}

void IceoryxChannelBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void IceoryxChannelBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  iceoryx_manager_.Shutdown();
}

bool IceoryxChannelBackend::RegisterPublishType(
    const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");
    namespace util = aimrt::common::util;

    const auto& info = publish_type_wrapper.info;
    std::string pattern = std::string("/channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type);

    // register publisher with url to iceoryx
    iceoryx_manager_.RegisterPublisher(pattern);

    AIMRT_INFO("Register publish type to iceoryx channel, url: {}", pattern);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool IceoryxChannelBackend::Subscribe(
    const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    namespace util = aimrt::common::util;

    const auto& info = subscribe_wrapper.info;

    std::string executor_name = options_.sub_default_executor;
    AIMRT_CHECK_ERROR_THROW(!executor_name.empty(), "iceoryx channel sub default executor not set!");

    auto find_option_itr = std::find_if(
        options_.sub_topics_options.begin(), options_.sub_topics_options.end(),
        [topic_name = info.topic_name](const Options::SubTopicOptions& sub_option) {
          try {
            return std::regex_match(topic_name.begin(), topic_name.end(), std::regex(sub_option.topic_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       sub_option.topic_name, topic_name, e.what());
            return false;
          }
        });

    if (find_option_itr != options_.sub_topics_options.end()) {
      executor_name = find_option_itr->executor;
    }

    std::string pattern = std::string("/channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type);

    auto find_itr = subscribe_wrapper_map_.find(pattern);
    if (find_itr != subscribe_wrapper_map_.end()) {
      find_itr->second->AddSubscribeWrapper(&subscribe_wrapper);
      return true;
    }

    // if not registered, register subscriber with url bind to handle to iceoryx
    auto sub_tool_unique_ptr = std::make_unique<runtime::core::channel::SubscribeTool>();
    sub_tool_unique_ptr->AddSubscribeWrapper(&subscribe_wrapper);

    auto* sub_tool_ptr = sub_tool_unique_ptr.get();

    subscribe_wrapper_map_.emplace(pattern, std::move(sub_tool_unique_ptr));

    auto handle =
        [this, topic_name = info.topic_name, sub_tool_ptr](iox::popo::UntypedSubscriber* subscriber) {
          try {
            // read data from shared memory : pkg_size | serialization_type | ctx_num | ctx_key1 | ctx_val1 | ... | ctx_keyN | ctx_valN | msg_buffer
            // use while struck to make sure all packages are read
            while (subscriber->hasData()) {
              subscriber->take()
                  .and_then([&](const void* payload) {
                    auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

                    uint32_t pkg_size = util::GetUint32FromBuf(static_cast<const char*>(payload));

                    util::ConstBufferOperator buf_oper(static_cast<const char*>(payload) + 4, pkg_size - 4);

                    // get serialization type
                    std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
                    ctx_ptr->SetSerializationType(serialization_type);

                    //  get context meta
                    size_t ctx_num = buf_oper.GetUint8();
                    for (size_t ii = 0; ii < ctx_num; ++ii) {
                      auto key = buf_oper.GetString(util::BufferLenType::kUInt16);
                      auto val = buf_oper.GetString(util::BufferLenType::kUInt16);
                      ctx_ptr->SetMetaValue(key, val);
                    }

                    ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

                    // get msg buffer
                    auto remaining_buf = buf_oper.GetRemainingBuffer();

                    sub_tool_ptr->DoSubscribeCallback(
                        ctx_ptr, serialization_type, static_cast<const void*>(remaining_buf.data()), remaining_buf.size());

                    // release shm
                    subscriber->release(payload);
                  })
                  .or_else([](auto& error) {
                    ;  // data has not been ready
                  });
            }
          } catch (const std::exception& e) {
            AIMRT_WARN("Handle Iceoryx channel msg failed, exception info: {}", e.what());
          }
        };

    iceoryx_manager_.RegisterSubscriber(pattern, executor_name, std::move(handle));

    AIMRT_INFO("Register subscribe type  to iceoryx channel, url: {}", pattern);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void IceoryxChannelBackend::Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kStart,
                            "Method can only be called when state is 'Start'.");

    namespace util = aimrt::common::util;

    const auto& info = msg_wrapper.info;

    std::string iceoryx_pub_topic = std::string("/channel/") +
                                    util::UrlEncode(info.topic_name) + "/" +
                                    util::UrlEncode(info.msg_type);

    // find publisher
    auto* iox_pub_ctx_ptr = iceoryx_manager_.GetPublisher(iceoryx_pub_topic);
    AIMRT_CHECK_ERROR_THROW(iox_pub_ctx_ptr != nullptr,
                            "Url: {} not registered for publishing!", iceoryx_pub_topic);

    auto publish_type_support_ref = info.msg_type_support_ref;

    // get serialization type
    std::string_view serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    if (serialization_type.empty()) {
      serialization_type = publish_type_support_ref.DefaultSerializationType();
    }

    // statistics context meta
    auto [meta_key_vals_array, meta_key_vals_array_len] = msg_wrapper.ctx_ref.GetMetaKeyValsArray();
    AIMRT_CHECK_ERROR_THROW(meta_key_vals_array_len / 2 <= 255,
                            "Too much context meta, require less than 255, but actually {}.", meta_key_vals_array_len / 2);

    size_t context_meta_kv_size = 1;
    for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
      context_meta_kv_size += (2 + meta_key_vals_array[ii].len);
    }

    // check serialization cache
    auto& serialization_cache = msg_wrapper.serialization_cache;
    auto finditr = serialization_cache.find(serialization_type);
    if (finditr != serialization_cache.end()) [[unlikely]] {
      // publish with cache
      auto buffer_array_view_ptr = finditr->second;

      const auto* buffer_array_data = buffer_array_view_ptr->Data();
      const size_t buffer_array_len = buffer_array_view_ptr->Size();
      size_t msg_size = buffer_array_view_ptr->BufferSize();

      size_t shn_size = 4 + 1 + serialization_type.size() + context_meta_kv_size + msg_size;
      auto loaned_shm = iox_pub_ctx_ptr->LoanShm(shn_size);
      util::BufferOperator buf_oper(static_cast<char*>(loaned_shm.Ptr()), loaned_shm.Size());

      buf_oper.SetUint32(shn_size);

      buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

      buf_oper.SetUint8(static_cast<uint8_t>(meta_key_vals_array_len / 2));
      for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
        buf_oper.SetString(aimrt::util::ToStdStringView(meta_key_vals_array[ii]), util::BufferLenType::kUInt16);
      }

      for (size_t ii = 0; ii < buffer_array_len; ++ii) {
        buf_oper.SetBuffer(
            static_cast<const char*>(buffer_array_data[ii].data),
            buffer_array_data[ii].len);
      }

      iox_pub_ctx_ptr->PublishShm(loaned_shm);

      AIMRT_TRACE("Iceoryx publish to '{}'", iceoryx_pub_topic);

      return;
    }

    // publish without cache
    CheckMsg(msg_wrapper);

    size_t min_shm_size = 4 + 1 + serialization_type.size() + context_meta_kv_size;
    auto loaned_shm = iox_pub_ctx_ptr->LoanShm(min_shm_size);

    // dynamic allocation for loaned shm
    while (true) {
      util::BufferOperator buf_oper(static_cast<char*>(loaned_shm.Ptr()), loaned_shm.Size());

      // skip pkg_size
      buf_oper.Skip(4);

      // write serialization type on loaned shm
      buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

      // write context meta on loaned shm
      buf_oper.SetUint8(static_cast<uint8_t>(meta_key_vals_array_len / 2));
      for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
        buf_oper.SetString(aimrt::util::ToStdStringView(meta_key_vals_array[ii]), util::BufferLenType::kUInt16);
      }

      // write msg on loaned shm
      aimrt::util::FlatBufferArrayAllocator allocator(
          static_cast<char*>(loaned_shm.Ptr()) + min_shm_size, loaned_shm.Size() - min_shm_size);
      aimrt::util::BufferArray buffer_array(allocator.NativeHandle());

      bool serialize_ret = info.msg_type_support_ref.Serialize(
          serialization_type,
          msg_wrapper.msg_ptr,
          buffer_array.AllocatorNativeHandle(),
          buffer_array.BufferArrayNativeHandle());

      if (!serialize_ret) [[unlikely]] {
        if (allocator.IsOutOfMemory()) {
          // release old shm and loan a new size shm
          iox_pub_ctx_ptr->UpdateLoanShm(loaned_shm, loaned_shm.Size() * 2);
          continue;
        }

        AIMRT_ERROR_THROW("Serialize failed.");
      }

      // write info pkg length on loaned shm
      buf_oper.JumpTo(0);
      buf_oper.SetUint32(min_shm_size + buffer_array.BufferSize());

      break;
    }

    iox_pub_ctx_ptr->PublishShm(loaned_shm);

    AIMRT_TRACE("Iceoryx publish to '{}'", iceoryx_pub_topic);

    return;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

}  // namespace aimrt::plugins::iceoryx_plugin