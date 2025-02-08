// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "zenoh_plugin/zenoh_channel_backend.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::zenoh_plugin::ZenohChannelBackend::Options> {
  using Options = aimrt::plugins::zenoh_plugin::ZenohChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["pub_topics_options"] = YAML::Node();
    for (const auto& pub_topic_options : rhs.pub_topics_options) {
      Node pub_topic_options_node;
      pub_topic_options_node["topic_name"] = pub_topic_options.topic_name;
      pub_topic_options_node["shm_enabled"] = pub_topic_options.shm_enabled;
      node["pub_topics_options"].push_back(pub_topic_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["pub_topics_options"] && node["pub_topics_options"].IsSequence()) {
      for (const auto& pub_topic_options_node : node["pub_topics_options"]) {
        auto pub_topic_options = Options::PubTopicOptions{
            .topic_name = pub_topic_options_node["topic_name"].as<std::string>(),
            .shm_enabled = pub_topic_options_node["shm_enabled"].as<bool>(),
        };

        rhs.pub_topics_options.emplace_back(std::move(pub_topic_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::zenoh_plugin {

void ZenohChannelBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Zenoh channel backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  options_node = options_;
}

void ZenohChannelBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");
}

void ZenohChannelBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;
}

bool ZenohChannelBackend::RegisterPublishType(
    const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    const auto& info = publish_type_wrapper.info;

    bool shm_enabled = false;

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
      shm_enabled = find_option_itr->shm_enabled;
    }

    namespace util = aimrt::common::util;
    std::string pattern = std::string("channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type) +
                          limit_domain_;

    zenoh_manager_ptr_->RegisterPublisher(pattern, shm_enabled);
    SetPubRegistry();
    z_pub_shm_size_map_[pattern] = shm_init_loan_size_;

    AIMRT_INFO("Register publish type to zenoh channel, url: {}, shm_enabled: {}", pattern, shm_enabled);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool ZenohChannelBackend::Subscribe(
    const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    const auto& info = subscribe_wrapper.info;
    namespace util = aimrt::common::util;

    std::string pattern = std::string("channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type) +
                          limit_domain_;

    auto find_itr = subscribe_wrapper_map_.find(pattern);
    if (find_itr != subscribe_wrapper_map_.end()) {
      find_itr->second->AddSubscribeWrapper(&subscribe_wrapper);
      return true;
    }

    auto sub_tool_unique_ptr = std::make_unique<aimrt::runtime::core::channel::SubscribeTool>();
    sub_tool_unique_ptr->AddSubscribeWrapper(&subscribe_wrapper);

    auto* sub_tool_ptr = sub_tool_unique_ptr.get();

    subscribe_wrapper_map_.emplace(pattern, std::move(sub_tool_unique_ptr));

    auto handle =
        [this, topic_name = info.topic_name, sub_tool_ptr](const z_loaned_sample_t* message) {
          try {
            auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);

            // todo(hj): avoid copy when use shm
            const z_loaned_bytes_t* payload = z_sample_payload(message);
            size_t serialized_size = z_bytes_len(payload);
            z_bytes_reader_t reader = z_bytes_get_reader(payload);
            std::vector<char> serialized_data(serialized_size);

            // read data from payload
            auto ret = z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(serialized_data.data()), serialized_size);
            if (ret >= 0) {
              util::ConstBufferOperator buf_oper_tmp(serialized_data.data(), 4);
              uint32_t serialized_size_with_len = buf_oper_tmp.GetUint32();

              util::ConstBufferOperator buf_oper(serialized_data.data() + 4, serialized_size_with_len);

              // get serialization type
              std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
              ctx_ptr->SetSerializationType(serialization_type);

              // get context meta
              size_t ctx_num = buf_oper.GetUint8();
              for (size_t ii = 0; ii < ctx_num; ++ii) {
                auto key = buf_oper.GetString(util::BufferLenType::kUInt16);
                auto val = buf_oper.GetString(util::BufferLenType::kUInt16);
                ctx_ptr->SetMetaValue(key, val);
              }
              ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

              // get msg
              auto remaining_buf = buf_oper.GetRemainingBuffer();

              sub_tool_ptr->DoSubscribeCallback(
                  ctx_ptr, serialization_type, static_cast<const void*>(remaining_buf.data()), remaining_buf.size());

            } else {
              AIMRT_ERROR("Zenoh Plugin Read payload failed!");
            }
          } catch (const std::exception& e) {
            AIMRT_WARN("Handle Zenoh channel msg failed, exception info: {}", e.what());
          }
        };

    zenoh_manager_ptr_->RegisterSubscriber(pattern, std::move(handle));
    AIMRT_INFO("Register subscribe type to zenoh channel, url: {}", pattern);
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void ZenohChannelBackend::Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept {
  try {
    // checkout state
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kStart,
                            "Method can only be called when state is 'Start'.");

    namespace util = aimrt::common::util;
    const auto& info = msg_wrapper.info;

    std::string zenoh_pub_topic = std::string("channel/") +
                                  util::UrlEncode(info.topic_name) + "/" +
                                  util::UrlEncode(info.msg_type) +
                                  limit_domain_;

    // find publisher with zenoh_pub_topic

    auto z_pub_iter = zenoh_pub_registry_ptr_->find(zenoh_pub_topic);
    if (z_pub_iter == zenoh_pub_registry_ptr_->end()) {
      AIMRT_ERROR("Url: {} is not registered!", zenoh_pub_topic);
      return;
    }

    // get serialization type
    auto publish_type_support_ref = info.msg_type_support_ref;

    auto serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    if (serialization_type.empty()) {
      serialization_type = publish_type_support_ref.DefaultSerializationType();
    }

    // get meta data
    const auto& keys = msg_wrapper.ctx_ref.GetMetaKeys();
    AIMRT_CHECK_ERROR_THROW(keys.size() <= 255,
                            "Too much context meta, require less than 255, but actually {}.", keys.size());

    std::vector<std::string_view> context_meta_kv;
    size_t context_meta_kv_size = 1;
    for (const auto& key : keys) {
      context_meta_kv_size += (2 + key.size());
      context_meta_kv.emplace_back(key);

      auto val = msg_wrapper.ctx_ref.GetMetaValue(key);
      context_meta_kv_size += (2 + val.size());
      context_meta_kv.emplace_back(val);
    }

    auto z_pub = z_pub_iter->second;
    // shm_enabled
    if (z_pub.second) {
      unsigned char* z_pub_loaned_shm_ptr = nullptr;
      std::shared_ptr<aimrt::util::BufferArrayView> buffer_array_cache_ptr = nullptr;

      bool is_shm_loan_size_enough = true;
      bool is_shm_pool_size_enough = true;

      uint64_t msg_size = 0;
      z_buf_layout_alloc_result_t loan_result;

      do {
        // release old shm
        if (z_pub_loaned_shm_ptr != nullptr) {
          z_shm_provider_garbage_collect(z_loan(zenoh_manager_ptr_->shm_provider_));
          z_shm_provider_defragment(z_loan(zenoh_manager_ptr_->shm_provider_));
        }

        // load a new size shm
        uint64_t loan_size = z_pub_shm_size_map_[zenoh_pub_topic];
        z_shm_provider_alloc(&loan_result, z_loan(zenoh_manager_ptr_->shm_provider_), loan_size, zenoh_manager_ptr_->alignment_);

        // if shm pool is not enough, use net buffer instead
        if (loan_result.status != ZC_BUF_LAYOUT_ALLOC_STATUS_OK) {
          is_shm_pool_size_enough = false;
          z_shm_provider_garbage_collect(z_loan(zenoh_manager_ptr_->shm_provider_));
          z_shm_provider_defragment(z_loan(zenoh_manager_ptr_->shm_provider_));
          AIMRT_WARN("Zenoh Plugin shm pool is not enough, use net buffer instead.");
          break;
        }

        z_pub_loaned_shm_ptr = z_shm_mut_data_mut(z_loan_mut(loan_result.buf));

        // write info pkg on loaned shm : the first FIXED_LEN bytes needs to write the length of pkg
        util::BufferOperator buf_oper(reinterpret_cast<char*>(z_pub_loaned_shm_ptr) + 4, loan_size - 4);

        // write serialization type on loaned shm
        buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

        // write context meta on loaned shm
        buf_oper.SetUint8(static_cast<uint8_t>(keys.size()));
        for (const auto& s : context_meta_kv) {
          buf_oper.SetString(s, util::BufferLenType::kUInt16);
        }
        auto type_and_ctx_len = 1 + serialization_type.size() + context_meta_kv_size;

        // write msg on loaned shm： should start at the (FIXED_LEN + type_and_ctx_len)-th byte
        aimrt::util::ZenohBufferArrayAllocator z_allocator(buf_oper.GetRemainingSize(), z_pub_loaned_shm_ptr + type_and_ctx_len + 4);
        if (buffer_array_cache_ptr == nullptr) {
          try {
            auto result = SerializeMsgSupportedZenoh(msg_wrapper, serialization_type, aimrt::util::BufferArrayAllocatorRef(z_allocator.NativeHandle()));
            msg_size = result.second;
            buffer_array_cache_ptr = result.first;
            if (buffer_array_cache_ptr == nullptr) {
              // in this case means no cache is set, then do nomal serialization(if size is small will throw exception)
              is_shm_loan_size_enough = true;
            } else {
              if (msg_size > buf_oper.GetRemainingSize()) {
                // in this case means the msg has serialization cache but the size is too large, then expand suitable size
                is_shm_loan_size_enough = false;
                z_pub_shm_size_map_[zenoh_pub_topic] = msg_size + type_and_ctx_len + 4;
              } else {
                // in this case means the msg has serialization cache and the size is suitable, then use cachema
                is_shm_loan_size_enough = true;
              }
            }

          } catch (const std::exception& e) {
            if (!z_allocator.IsShmEnough()) {
              // the shm is not enough, need to expand a double size
              z_pub_shm_size_map_[zenoh_pub_topic] = z_pub_shm_size_map_[zenoh_pub_topic] << 1;
              is_shm_loan_size_enough = false;
            } else {
              AIMRT_ERROR(
                  "Msg serialization failed, serialization_type {}, pkg_path: {}, module_name: {}, topic_name: {}, msg_type: {}, exception: {}",
                  serialization_type, info.pkg_path, info.module_name, info.topic_name, info.msg_type, e.what());
              return;
            }
          }
        }

      } while (!is_shm_loan_size_enough);

      if (is_shm_pool_size_enough) {
        // if has cache, the copy it to shm to replace the serialization
        if (buffer_array_cache_ptr != nullptr) {
          unsigned char* strat_pos = z_pub_loaned_shm_ptr + 4 + context_meta_kv_size + serialization_type.size() + 1;
          for (size_t ii = 0; ii < buffer_array_cache_ptr->Size(); ++ii) {
            std::memcpy(strat_pos, buffer_array_cache_ptr.get()[ii].Data()->data, buffer_array_cache_ptr.get()[ii].Data()->len);
            strat_pos += buffer_array_cache_ptr.get()[ii].Data()->len;
          }

          buffer_array_cache_ptr = nullptr;
        }
        // write info pkg length on loaned shm
        uint32_t data_size = 1 + serialization_type.size() + context_meta_kv_size + msg_size;
        util::SetBufFromUint32(reinterpret_cast<char*>(z_pub_loaned_shm_ptr), data_size);

        z_owned_bytes_t z_payload;
        if (loan_result.status == ZC_BUF_LAYOUT_ALLOC_STATUS_OK) {
          z_bytes_from_shm_mut(&z_payload, z_move(loan_result.buf));
        }
        z_publisher_put(z_loan(z_pub.first), z_move(z_payload), &zenoh_manager_ptr_->z_pub_options_);

        // collect garbage and defragment shared memory, whose reference counting is zero
        z_shm_provider_garbage_collect(z_loan(zenoh_manager_ptr_->shm_provider_));
        z_shm_provider_defragment(z_loan(zenoh_manager_ptr_->shm_provider_));

        AIMRT_TRACE("Zenoh publish to '{}'", zenoh_pub_topic);

        return;
      }
    }

    // shm disabled
    // serialize msg
    auto buffer_array_view_ptr = aimrt::runtime::core::channel::SerializeMsgWithCache(msg_wrapper, serialization_type);
    AIMRT_CHECK_ERROR_THROW(
        buffer_array_view_ptr,
        "Msg serialization failed, serialization_type {}, pkg_path: {}, module_name: {}, topic_name: {}, msg_type: {}",
        serialization_type, info.pkg_path, info.module_name, info.topic_name, info.msg_type);

    const auto* buffer_array_data = buffer_array_view_ptr->Data();
    const size_t buffer_array_len = buffer_array_view_ptr->Size();
    size_t msg_size = buffer_array_view_ptr->BufferSize();

    int32_t data_size = 1 + serialization_type.size() + context_meta_kv_size + msg_size;
    int32_t pkg_size = data_size + 4;

    // create buffer for serialization
    std::vector<char> serialized_data(pkg_size);
    util::BufferOperator buf_oper(serialized_data.data(), pkg_size);

    // full data_size
    buf_oper.SetUint32(data_size);

    // full serialization_type
    buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

    // full context meta
    buf_oper.SetUint8(static_cast<uint8_t>(keys.size()));
    for (const auto& s : context_meta_kv) {
      buf_oper.SetString(s, util::BufferLenType::kUInt16);
    }

    // full msg
    for (size_t ii = 0; ii < buffer_array_len; ++ii) {
      buf_oper.SetBuffer(
          static_cast<const char*>(buffer_array_data[ii].data),
          buffer_array_data[ii].len);
    }

    // publish
    z_owned_bytes_t z_payload;
    z_bytes_from_buf(&z_payload, reinterpret_cast<uint8_t*>(serialized_data.data()), pkg_size, nullptr, nullptr);
    z_publisher_put(z_loan(z_pub.first), z_move(z_payload), &zenoh_manager_ptr_->z_pub_options_);

    AIMRT_TRACE("Zenoh publish to '{}'", zenoh_pub_topic);

    return;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

}  // namespace aimrt::plugins::zenoh_plugin