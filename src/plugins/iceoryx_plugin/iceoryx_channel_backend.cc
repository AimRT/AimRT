// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "iceoryx_plugin/iceoryx_channel_backend.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::iceoryx_plugin::IceoryxChannelBackend::Options> {
  using Options = aimrt::plugins::iceoryx_plugin::IceoryxChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::iceoryx_plugin {
void IceoryxChannelBackend::Initialize(YAML::Node options_node) {
  // todo: check options_node->shm_init_size
  if (iox_shm_init_size_ == 0) {
    iox_shm_init_size_ = kIoxShmInitSize;
    AIMRT_INFO("Iceoryx shared memory init size is set to default value: {} bytes", kIoxShmInitSize);
  }
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
}

bool IceoryxChannelBackend::RegisterPublishType(
    const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    const auto& info = publish_type_wrapper.info;

    // todo: url check, each part should not exceed iox::MAX_RUNTIME_NAME_LENGTH(=100)
    namespace util = aimrt::common::util;
    std::string pattern = std::string("/channel/") +
                          util::UrlEncode(info.topic_name) + "/" +
                          util::UrlEncode(info.msg_type);

    // register publisher with url to iceoryx
    iceoryx_manager_ptr_->RegisterPublisher(pattern);
    iox_pub_shm_size_map_[pattern] = iox_shm_init_size_;

    AIMRT_INFO("Register publish type to iceoryx channel, url: {}, shm_init_size: {} bytes", pattern, iox_shm_init_size_);

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

    const auto& info = subscribe_wrapper.info;
    namespace util = aimrt::common::util;

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
            auto ctx_ptr = std::make_shared<aimrt::channel::Context>(aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);
            const char* msg = nullptr;

            // read data from shared memory : pkg_size | serialization_type | ctx_num | ctx_key1 | ctx_val1 | ... | ctx_keyN | ctx_valN | msg_buffer
            // use while struck to make sure all packages are read
            while (subscriber->take()
                       .and_then([&](const void* payload) {
                         msg = static_cast<const char*>(payload);

                         // fetch a data packet of a specified length
                         util::ConstBufferOperator buf_oper_tmp(msg, 4);
                         uint32_t pkg_size_with_len = buf_oper_tmp.GetUint32();

                         util::ConstBufferOperator buf_oper(msg + 4, pkg_size_with_len);

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
                       .or_else([&](auto& error) {
                         ;  // data has not been ready
                       })) {
            }
          } catch (const std::exception& e) {
            AIMRT_WARN("Handle Iceoryx channel msg failed, exception info: {}", e.what());
          }
        };

    iceoryx_manager_ptr_->RegisterSubscriber(pattern, std::move(handle));
    AIMRT_INFO("Register subscribe type  to iceoryx channel, url: {}, shm_init_size: {} bytes", pattern, iox_shm_init_size_);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

// dynamic allocation for loaned shm:
//
//         .---------------if not enough -----------------.
//         |                                              |
//         v                                              |
// release old shm   ——> loan double size   ——> try to write msg on shm  ——> if enough then publish
//
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
    auto iox_pub_registry = iceoryx_manager_ptr_->GetPublisherRegisterMap();
    auto iox_pub_iter = iox_pub_registry->find(iceoryx_pub_topic);
    if (iox_pub_iter == iox_pub_registry->end()) {
      AIMRT_ERROR("Url: {} not registered for publishing!", iceoryx_pub_topic);
      return;
    }
    auto iox_pub = iox_pub_iter->second;

    auto publish_type_support_ref = info.msg_type_support_ref;

    // get serialization type
    std::string_view serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    if (serialization_type.empty()) {
      serialization_type = publish_type_support_ref.DefaultSerializationType();
    }

    // statistics context meta
    const auto& keys = msg_wrapper.ctx_ref.GetMetaKeys();
    if (keys.size() > 255) [[unlikely]] {
      AIMRT_WARN("Too much context meta, require less than 255, but actually {}.", keys.size());
      return;
    }

    std::vector<std::string_view> context_meta_kv;
    size_t context_meta_kv_size = 1;
    for (const auto& key : keys) {
      context_meta_kv_size += (2 + key.size());
      context_meta_kv.emplace_back(key);

      auto val = msg_wrapper.ctx_ref.GetMetaValue(key);
      context_meta_kv_size += (2 + val.size());
      context_meta_kv.emplace_back(val);
    }

    void* iox_pub_loaned_shm_ptr = nullptr;
    std::shared_ptr<aimrt::util::BufferArrayView> buffer_array_cache_ptr = nullptr;

    bool is_shm_enough = true;
    uint64_t msg_size = 0;

    {
      // avoid muti-thread write the same shm casuing write-fault
      std::lock_guard<std::mutex> lock(iox_write_mutex_);
      do {
        uint64_t loan_size = iox_pub_shm_size_map_[iceoryx_pub_topic];

        // release old shm
        if (iox_pub_loaned_shm_ptr != NULL) {
          iox_pub->release(iox_pub_loaned_shm_ptr);
        }

        // load a new size shm
        auto loan_result = iox_pub->loan(loan_size);
        iox_pub_loaned_shm_ptr = loan_result.value();

        // write info pkg on loaned shm : the first FIXED_LEN bytes needs to write the length of pkg
        util::BufferOperator buf_oper(reinterpret_cast<char*>(iox_pub_loaned_shm_ptr) + 4, loan_size - 4);

        // write serialization type on loaned shm
        buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

        // write context meta on loaned shm
        buf_oper.SetUint8(static_cast<uint8_t>(keys.size()));
        for (const auto& s : context_meta_kv) {
          buf_oper.SetString(s, util::BufferLenType::kUInt16);
        }

        auto type_and_ctx_len = 1 + serialization_type.size() + context_meta_kv_size;

        // write msg on loaned shm： should start at the (FIXED_LEN + type_and_ctx_len)-th byte
        aimrt::util::IceoryxBufferArrayAllocator iox_allocator(buf_oper.GetRemainingSize(), static_cast<char*>(iox_pub_loaned_shm_ptr) + type_and_ctx_len + 4);
        if (buffer_array_cache_ptr == nullptr) {
          try {
            auto result = SerializeMsgSupportedIceoryx(msg_wrapper, serialization_type, aimrt::util::BufferArrayAllocatorRef(iox_allocator.NativeHandle()));
            msg_size = result.second;
            buffer_array_cache_ptr = result.first;

            if (buffer_array_cache_ptr == nullptr) {
              // in this case means no cache is set, then do nomal serialization(if size is small will throw exception)
              is_shm_enough = true;
            } else {
              if (msg_size > buf_oper.GetRemainingSize()) {
                // in this case means the msg has serialization cache but the size is too large, then expand suitable size
                is_shm_enough = false;
                iox_pub_shm_size_map_[iceoryx_pub_topic] = msg_size + type_and_ctx_len + 4;

              } else {
                // in this case means the msg has serialization cache and the size is suitable, then use cachema
                is_shm_enough = true;
              }
            }

          } catch (const std::exception& e) {
            if (!iox_allocator.IsShmEnough()) {
              // the shm is not enough, need to expand a double size
              iox_pub_shm_size_map_[iceoryx_pub_topic] = iox_pub_shm_size_map_[iceoryx_pub_topic] << 1;
              is_shm_enough = false;
            } else {
              AIMRT_ERROR(
                  "Msg serialization failed, serialization_type {}, pkg_path: {}, module_name: {}, topic_name: {}, msg_type: {}, exception: {}",
                  serialization_type, info.pkg_path, info.module_name, info.topic_name, info.msg_type, e.what());
              return;
            }
          }
        }
      } while (!is_shm_enough);

      // if has cache, the copy it to shm to replace the serialization
      if (buffer_array_cache_ptr != nullptr) {
        char* strat_pos = static_cast<char*>(iox_pub_loaned_shm_ptr) + 4 + context_meta_kv_size + serialization_type.size() + 1;
        for (size_t ii = 0; ii < buffer_array_cache_ptr->Size(); ++ii) {
          std::memcpy(strat_pos, buffer_array_cache_ptr.get()[ii].Data()->data, buffer_array_cache_ptr.get()[ii].Data()->len);
          strat_pos += buffer_array_cache_ptr.get()[ii].Data()->len;
        }

        buffer_array_cache_ptr = nullptr;
      }

      // write info pkg length on loaned shm
      uint32_t data_size = 1 + serialization_type.size() + context_meta_kv_size + msg_size;
      std::memcpy(static_cast<char*>(iox_pub_loaned_shm_ptr), &data_size, 4);

      iox_pub->publish(iox_pub_loaned_shm_ptr);
    }

    AIMRT_TRACE("Iceoryx publish to '{}'", iceoryx_pub_topic);

    return;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

}  // namespace aimrt::plugins::iceoryx_plugin