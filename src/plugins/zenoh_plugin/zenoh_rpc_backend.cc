// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "zenoh_plugin/zenoh_rpc_backend.h"

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::zenoh_plugin::ZenohRpcBackend::Options> {
  using Options = aimrt::plugins::zenoh_plugin::ZenohRpcBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["timeout_executor"] = rhs.timeout_executor;

    node["clients_options"] = YAML::Node();
    for (const auto& client_options : rhs.clients_options) {
      Node client_options_node;
      client_options_node["func_name"] = client_options.func_name;
      client_options_node["shm_enabled"] = client_options.shm_enabled;
      node["clients_options"].push_back(client_options_node);
    }

    node["servers_options"] = YAML::Node();
    for (const auto& server_options : rhs.servers_options) {
      Node server_options_node;
      server_options_node["func_name"] = server_options.func_name;
      server_options_node["shm_enabled"] = server_options.shm_enabled;
      node["servers_options"].push_back(server_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["timeout_executor"])
      rhs.timeout_executor = node["timeout_executor"].as<std::string>();

    if (node["clients_options"] && node["clients_options"].IsSequence()) {
      for (const auto& client_options_node : node["clients_options"]) {
        auto client_options = Options::ClientOptions{
            .func_name = client_options_node["func_name"].as<std::string>(),
            .shm_enabled = client_options_node["shm_enabled"].as<bool>(),
        };

        rhs.clients_options.emplace_back(std::move(client_options));
      }
    }

    if (node["servers_options"] && node["servers_options"].IsSequence()) {
      for (const auto& server_options_node : node["servers_options"]) {
        auto server_options = Options::ServerOptions{
            .func_name = server_options_node["func_name"].as<std::string>(),
            .shm_enabled = server_options_node["shm_enabled"].as<bool>(),
        };

        rhs.servers_options.emplace_back(std::move(server_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::zenoh_plugin {

void ZenohRpcBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::Init) == State::PreInit,
      "Zenoh Rpc backend can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  client_tool_ptr_ =
      std::make_unique<runtime::core::util::RpcClientTool<std::shared_ptr<runtime::core::rpc::InvokeWrapper>>>();

  if (!options_.timeout_executor.empty()) {
    AIMRT_CHECK_ERROR_THROW(
        get_executor_func_,
        "Get executor function is not set before initialize.");

    auto timeout_executor = get_executor_func_(options_.timeout_executor);

    AIMRT_CHECK_ERROR_THROW(
        timeout_executor,
        "Get timeout executor '{}' failed.", options_.timeout_executor);

    client_tool_ptr_->RegisterTimeoutExecutor(timeout_executor);
    client_tool_ptr_->RegisterTimeoutHandle(
        [](auto&& client_invoke_wrapper_ptr) {
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_TIMEOUT));
        });

    AIMRT_TRACE("zenoh rpc backend enable the timeout function, use '{}' as timeout executor.",
                options_.timeout_executor);
  } else {
    AIMRT_TRACE("zenoh rpc backend does not enable the timeout function.");
  }
}

void ZenohRpcBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::Start) == State::Init,
      "Method can only be called when state is 'Init'.");
}

void ZenohRpcBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::Shutdown) == State::Shutdown)
    return;

  client_tool_ptr_.reset();
}

bool ZenohRpcBackend::RegisterServiceFunc(
    const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper) noexcept {
  try {
    if (state_.load() != State::Init) {
      AIMRT_ERROR("Service func can only be registered when state is 'Init'.");
      return false;
    }

    namespace util = aimrt::common::util;
    const auto& info = service_func_wrapper.info;

    bool shm_enabled = false;

    auto find_option_itr = std::find_if(
        options_.servers_options.begin(), options_.servers_options.end(),
        [func_name = info.func_name](const Options::ServerOptions& server_option) {
          try {
            auto real_func_name = std::string(rpc::GetFuncNameWithoutPrefix(func_name));
            return std::regex_match(func_name.begin(), func_name.end(),
                                    std::regex(server_option.func_name, std::regex::ECMAScript)) ||
                   std::regex_match(real_func_name.begin(), real_func_name.end(),
                                    std::regex(server_option.func_name, std::regex::ECMAScript));
          } catch (const std::exception& e) {
            AIMRT_WARN("Regex get exception, expr: {}, string: {}, exception info: {}",
                       server_option.func_name, func_name, e.what());
            return false;
          }
        });

    if (find_option_itr != options_.servers_options.end()) {
      shm_enabled = find_option_itr->shm_enabled;
    }

    std::string pattern = std::string("aimrt_rpc/") +
                          util::UrlEncode(rpc::GetFuncNameWithoutPrefix(info.func_name)) +
                          limit_domain_;

    auto handle = [this, &service_func_wrapper, pattern](const z_loaned_sample_t* message) {
      try {
        // create service invoke wrapper
        auto service_invoke_wrapper_ptr = std::make_shared<runtime::core::rpc::InvokeWrapper>(
            runtime::core::rpc::InvokeWrapper{.info = service_func_wrapper.info});
        const auto& info = service_invoke_wrapper_ptr->info;

        // create ctx
        auto ctx_ptr = std::make_shared<aimrt::rpc::Context>(aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT);
        service_invoke_wrapper_ptr->ctx_ref = ctx_ptr;

        const z_loaned_bytes_t* payload = z_sample_payload(message);
        size_t serialized_size = z_bytes_len(payload);
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        std::vector<char> serialized_data(serialized_size);

        // read data from payload
        auto ret = z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(serialized_data.data()), serialized_size);
        if (ret >= 0) {
          util::ConstBufferOperator buf_oper_tmp(serialized_data.data(), 4);
          uint32_t pkg_size_with_len = buf_oper_tmp.GetUint32();

          util::ConstBufferOperator buf_oper(serialized_data.data() + 4, pkg_size_with_len);

          // deserialize type
          std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
          ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE, serialization_type);

          std::string pattern(buf_oper.GetString(util::BufferLenType::kUInt8));
          char req_id_buf[4];
          buf_oper.GetBuffer(req_id_buf, 4);

          size_t ctx_num = buf_oper.GetUint8();
          for (size_t ii = 0; ii < ctx_num; ++ii) {
            auto key = buf_oper.GetString(util::BufferLenType::kUInt16);
            auto val = buf_oper.GetString(util::BufferLenType::kUInt16);
            ctx_ptr->SetMetaValue(key, val);
          }
          ctx_ptr->SetFunctionName(info.func_name);
          ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND, Name());

          // deserialize msg
          auto remaining_buf = buf_oper.GetRemainingBuffer();
          aimrt_buffer_view_t buffer_view{
              .data = remaining_buf.data(),
              .len = remaining_buf.size()};

          aimrt_buffer_array_view_t buffer_array_view{
              .data = &buffer_view,
              .len = 1};

          std::shared_ptr<void> service_req_ptr = info.req_type_support_ref.CreateSharedPtr();
          service_invoke_wrapper_ptr->req_ptr = service_req_ptr.get();

          bool deserialize_ret = info.req_type_support_ref.Deserialize(
              serialization_type, buffer_array_view, service_req_ptr.get());

          AIMRT_CHECK_ERROR_THROW(deserialize_ret, "Zenoh req deserialize failed.");

          // create service rsp
          std::shared_ptr<void> service_rsp_ptr = info.rsp_type_support_ref.CreateSharedPtr();
          service_invoke_wrapper_ptr->rsp_ptr = service_rsp_ptr.get();

          // set callback
          service_invoke_wrapper_ptr->callback =
              [this,
               service_invoke_wrapper_ptr,
               ctx_ptr,
               service_req_ptr,
               service_rsp_ptr,
               serialization_type{std::move(serialization_type)},
               pattern{std::move(pattern)},
               req_id_buf](aimrt::rpc::Status status) {
                if (!status.OK()) [[unlikely]] {
                  ReturnRspWithStatusCode(
                      pattern, serialization_type, req_id_buf, status.Code());
                  return;
                }

                std::string node_pub_topic = "rsp/" + pattern;

                // find node's publisher with pattern
                const auto& zenoh_pub_registry_ptr = zenoh_manager_ptr_->GetPublisherRegisterMap();
                auto z_node_pub_iter = zenoh_pub_registry_ptr.find(node_pub_topic);
                if (z_node_pub_iter == zenoh_pub_registry_ptr.end()) [[unlikely]] {
                  AIMRT_ERROR("Can not find publisher with pattern: {}", pattern);
                  return;
                }

                auto z_pub_ctx_ptr = z_node_pub_iter->second;

                // shm enabled
                if (z_pub_ctx_ptr->shm_enabled) {
                  unsigned char* z_pub_loaned_shm_ptr = nullptr;
                  std::shared_ptr<aimrt::util::BufferArrayView> buffer_array_cache_ptr = nullptr;

                  bool is_shm_loan_size_enough = true;
                  bool is_shm_pool_size_enough = true;

                  uint32_t msg_size = 0;
                  uint32_t header_len = 0;
                  z_buf_layout_alloc_result_t loan_result;

                  do {
                    // release old shm
                    if (z_pub_loaned_shm_ptr != nullptr) {
                      z_shm_provider_garbage_collect(z_loan(zenoh_manager_ptr_->shm_provider_));
                      z_shm_provider_defragment(z_loan(zenoh_manager_ptr_->shm_provider_));
                    }

                    // loan a new size shm
                    uint64_t loan_size = 0;
                    {
                      std::shared_lock lock(z_shared_mutex_);
                      loan_size = z_node_shm_size_map_[node_pub_topic];
                    }
                    z_shm_provider_alloc_gc_defrag(&loan_result, z_loan(zenoh_manager_ptr_->shm_provider_), loan_size, zenoh_manager_ptr_->alignment_);

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

                    // write req id on loaned shm
                    buf_oper.SetBuffer(req_id_buf, sizeof(req_id_buf));

                    // write an zero on loaned shm
                    buf_oper.SetUint32(0);

                    header_len = 1 + serialization_type.size() + 4 + 4;

                    // write msg on loaned shm： should start at the (FIXED_LEN + header_len)-th byte
                    aimrt::util::ZenohBufferArrayAllocator z_allocator(buf_oper.GetRemainingSize(), z_pub_loaned_shm_ptr + header_len + 4);

                    if (buffer_array_cache_ptr == nullptr) {
                      try {
                        auto result = SerializeRspSupportedZenoh(*service_invoke_wrapper_ptr, serialization_type, aimrt::util::BufferArrayAllocatorRef(z_allocator.NativeHandle()));
                        msg_size = result.second;
                        buffer_array_cache_ptr = result.first;
                        if (buffer_array_cache_ptr == nullptr) {
                          // in this case means no cache is set, then do nomal serialization(if size is small will throw exception)
                          is_shm_loan_size_enough = true;
                        } else {
                          if (msg_size > buf_oper.GetRemainingSize()) {
                            // in this case means the msg has serialization cache but the size is too large, then expand suitable size
                            is_shm_loan_size_enough = false;
                            {
                              std::unique_lock lock(z_shared_mutex_);
                              z_node_shm_size_map_[node_pub_topic] = 4 + header_len + msg_size;
                            }
                          } else {
                            // in this case means the msg has serialization cache and the size is suitable, then use cachema
                            is_shm_loan_size_enough = true;
                          }
                        }

                      } catch (const std::exception& e) {
                        if (!z_allocator.IsShmEnough()) {
                          // the shm is not enough, need to expand a double size
                          {
                            std::unique_lock lock(z_shared_mutex_);
                            auto& ret = z_node_shm_size_map_[node_pub_topic];
                            ret = ret << 1;
                          }
                          is_shm_loan_size_enough = false;
                        } else {
                          AIMRT_ERROR(
                              "Msg serialization failed, serialization_type {}, pattern: {}, exception: {}",
                              serialization_type, pattern, e.what());
                          return;
                        }
                      }
                    }

                  } while (!is_shm_loan_size_enough);

                  if (is_shm_pool_size_enough) {
                    // if has cache, the copy it to shm to replace the serialization
                    if (buffer_array_cache_ptr != nullptr) {
                      unsigned char* strat_pos = z_pub_loaned_shm_ptr + 4 + header_len;
                      for (size_t ii = 0; ii < buffer_array_cache_ptr->Size(); ++ii) {
                        std::memcpy(strat_pos, buffer_array_cache_ptr.get()[ii].Data()->data, buffer_array_cache_ptr.get()[ii].Data()->len);
                        strat_pos += buffer_array_cache_ptr.get()[ii].Data()->len;
                      }

                      buffer_array_cache_ptr = nullptr;
                    }

                    // write info pkg length on loaned shm
                    util::SetBufFromUint32(reinterpret_cast<char*>(z_pub_loaned_shm_ptr), header_len);

                    z_owned_bytes_t z_payload;
                    if (loan_result.status == ZC_BUF_LAYOUT_ALLOC_STATUS_OK) {
                      z_bytes_from_shm_mut(&z_payload, z_move(loan_result.buf));
                    }
                    z_publisher_put(z_loan(z_pub_ctx_ptr->z_pub), z_move(z_payload), &zenoh_manager_ptr_->z_pub_options_);

                    // collect garbage and defragment shared memory, whose reference counting is zero
                    z_shm_provider_garbage_collect(z_loan(zenoh_manager_ptr_->shm_provider_));
                    z_shm_provider_defragment(z_loan(zenoh_manager_ptr_->shm_provider_));

                    AIMRT_TRACE("Zenoh Invoke req  with '{}'", pattern);
                    return;
                  }
                }

                // shm disabled
                // serivice rsp serialize
                auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeRspWithCache(
                    *service_invoke_wrapper_ptr, serialization_type);
                if (!buffer_array_view_ptr) [[unlikely]] {
                  ReturnRspWithStatusCode(
                      pattern, serialization_type, req_id_buf, AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED);

                  return;
                }
                const auto* buffer_array_data = buffer_array_view_ptr->Data();
                const size_t buffer_array_len = buffer_array_view_ptr->Size();
                size_t rsp_size = buffer_array_view_ptr->BufferSize();

                size_t z_data_size = 1 + serialization_type.size() + 4 + 4 + rsp_size;
                size_t pkg_size = z_data_size + 4;

                // get buf to store data
                std::vector<char> msg_buf_vec(pkg_size);
                util::BufferOperator buf_oper(msg_buf_vec.data(), pkg_size);

                // full data_size
                buf_oper.SetUint32(z_data_size);

                // full serialize type
                buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

                // full req_id
                buf_oper.SetBuffer(req_id_buf, sizeof(req_id_buf));

                // full an 0
                buf_oper.SetUint32(0);

                // full rsp_size
                for (size_t ii = 0; ii < buffer_array_len; ++ii) {
                  buf_oper.SetBuffer(
                      static_cast<const char*>(buffer_array_data[ii].data),
                      buffer_array_data[ii].len);
                }

                // server send rsp
                z_owned_bytes_t z_payload;
                z_bytes_from_buf(&z_payload, reinterpret_cast<uint8_t*>(msg_buf_vec.data()), pkg_size, nullptr, nullptr);
                z_publisher_put(z_loan(z_pub_ctx_ptr->z_pub), z_move(z_payload), &zenoh_manager_ptr_->z_pub_options_);
              };
          // call service
          service_func_wrapper.service_func(service_invoke_wrapper_ptr);

        } else {
          AIMRT_ERROR("Zenoh Plugin Read payload failed!");
        }
      } catch (const std::exception& e) {
        AIMRT_WARN("Handle zenoh rpc msg failed, exception info: {}", e.what());
      }
    };
    zenoh_manager_ptr_->RegisterRpcNode(pattern, std::move(handle), "server", shm_enabled);

    z_node_shm_size_map_["rsp/" + pattern] = shm_init_loan_size_;
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool ZenohRpcBackend::RegisterClientFunc(
    const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper) noexcept {
  try {
    if (state_.load() != State::Init) {
      AIMRT_ERROR("Client func can only be registered when state is 'Init'.");
      return false;
    }

    namespace util = aimrt::common::util;

    const auto& info = client_func_wrapper.info;

    bool shm_enabled = false;
    auto find_option_itr = std::find_if(
        options_.clients_options.begin(), options_.clients_options.end(),
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

    if (find_option_itr != options_.clients_options.end()) {
      shm_enabled = find_option_itr->shm_enabled;
    }

    std::string pattern = std::string("aimrt_rpc/") +
                          util::UrlEncode(rpc::GetFuncNameWithoutPrefix(info.func_name)) +
                          limit_domain_;
    auto handle = [this](const z_loaned_sample_t* message) {
      std::shared_ptr<runtime::core::rpc::InvokeWrapper> client_invoke_wrapper_ptr;
      try {
        // read data
        const z_loaned_bytes_t* payload = z_sample_payload(message);
        size_t serialized_size = z_bytes_len(payload);
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        std::vector<char> serialized_data(serialized_size);

        auto read_ret = z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(serialized_data.data()), serialized_size);
        if (read_ret < 0) {
          AIMRT_ERROR("Zenoh Plugin Read payload failed!");
          return;
        }

        util::ConstBufferOperator buf_oper_tmp(serialized_data.data(), 4);
        uint32_t data_size_with_len = buf_oper_tmp.GetUint32();

        util::ConstBufferOperator buf_oper(serialized_data.data() + 4, data_size_with_len);

        std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));
        uint32_t req_id = buf_oper.GetUint32();
        uint32_t code = buf_oper.GetUint32();

        auto msg_recorder = client_tool_ptr_->GetRecord(req_id);
        if (!msg_recorder) [[unlikely]] {
          // can't find recorder, which means timeout
          AIMRT_TRACE("Can not get req id {} from recorder.", req_id);
          return;
        }

        // find record
        client_invoke_wrapper_ptr = std::move(*msg_recorder);

        if (code) [[unlikely]] {
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(code));
          return;
        }

        const auto& info = client_invoke_wrapper_ptr->info;

        // client rsp deserialize
        auto remaining_buf = buf_oper.GetRemainingBuffer();
        aimrt_buffer_view_t buffer_view{
            .data = remaining_buf.data(),
            .len = remaining_buf.size()};

        aimrt_buffer_array_view_t buffer_array_view{
            .data = &buffer_view,
            .len = 1};

        bool deserialize_ret = info.rsp_type_support_ref.Deserialize(
            serialization_type, buffer_array_view, client_invoke_wrapper_ptr->rsp_ptr);

        if (!deserialize_ret) {
          // deserialize failed
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED));
          return;
        }

        client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_OK));
        return;
      } catch (const std::exception& e) {
        AIMRT_WARN("Handle zenoh rpc msg failed, exception info: {}", e.what());
      }

      if (client_invoke_wrapper_ptr)
        client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
    };

    zenoh_manager_ptr_->RegisterRpcNode(pattern, std::move(handle), "client", shm_enabled);

    z_node_shm_size_map_["req/" + pattern] = shm_init_loan_size_;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
  return true;
}

void ZenohRpcBackend::Invoke(
    const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) noexcept {
  try {
    if (state_.load() != State::Start) [[unlikely]] {
      AIMRT_WARN("Method can only be called when state is 'Start'.");
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    namespace util = aimrt::common::util;
    const auto& info = client_invoke_wrapper_ptr->info;

    std::string pattern = std::string("aimrt_rpc/") +
                          util::UrlEncode(rpc::GetFuncNameWithoutPrefix(info.func_name)) +
                          limit_domain_;

    std::string node_pub_topic = "req/" + pattern;

    // find node's publisher with pattern
    const auto& zenoh_pub_registry_ptr = zenoh_manager_ptr_->GetPublisherRegisterMap();
    auto z_node_pub_iter = zenoh_pub_registry_ptr.find(node_pub_topic);
    if (z_node_pub_iter == zenoh_pub_registry_ptr.end()) [[unlikely]] {
      AIMRT_ERROR("Can not find publisher with pattern: {}", pattern);
      return;
    }

    auto z_pub_ctx_ptr = z_node_pub_iter->second;

    // get req id
    uint32_t cur_req_id = req_id_++;

    // get serialization type
    auto serialization_type =
        client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE);

    if (serialization_type.size() > 255) [[unlikely]] {
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_UNKNOWN));
      return;
    }

    // get meta data
    const auto& keys = client_invoke_wrapper_ptr->ctx_ref.GetMetaKeys();
    if (keys.size() > 255) [[unlikely]] {
      AIMRT_WARN("Too much context meta, require less than 255, but actually {}.", keys.size());
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    std::vector<std::string_view> context_meta_kv;
    size_t context_meta_kv_size = 1;
    for (const auto& key : keys) {
      context_meta_kv_size += (2 + key.size());
      context_meta_kv.emplace_back(key);

      auto val = client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(key);
      context_meta_kv_size += (2 + val.size());
      context_meta_kv.emplace_back(val);
    }

    auto timeout = client_invoke_wrapper_ptr->ctx_ref.Timeout();
    auto record_ptr = client_invoke_wrapper_ptr;

    // record this req with timeout
    bool ret = client_tool_ptr_->Record(cur_req_id, timeout, std::move(record_ptr));

    if (!ret) [[unlikely]] {
      AIMRT_ERROR("Failed to record msg.");
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
      return;
    }

    // shm enabled
    if (z_pub_ctx_ptr->shm_enabled) {
      unsigned char* z_pub_loaned_shm_ptr = nullptr;
      std::shared_ptr<aimrt::util::BufferArrayView> buffer_array_cache_ptr = nullptr;

      bool is_shm_loan_size_enough = true;
      bool is_shm_pool_size_enough = true;

      uint32_t msg_size = 0;
      uint32_t header_len = 0;
      z_buf_layout_alloc_result_t loan_result;

      do {
        // release old shm
        if (z_pub_loaned_shm_ptr != nullptr) {
          z_shm_provider_garbage_collect(z_loan(zenoh_manager_ptr_->shm_provider_));
          z_shm_provider_defragment(z_loan(zenoh_manager_ptr_->shm_provider_));
        }

        // loan a new size shm
        uint64_t loan_size = 0;
        {
          std::shared_lock lock(z_shared_mutex_);
          loan_size = z_node_shm_size_map_[node_pub_topic];
        }
        z_shm_provider_alloc_gc_defrag(&loan_result, z_loan(zenoh_manager_ptr_->shm_provider_), loan_size, zenoh_manager_ptr_->alignment_);

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

        // write pattern on loaned shm
        buf_oper.SetString(pattern, util::BufferLenType::kUInt8);

        // write req id on loaned shm
        buf_oper.SetUint32(cur_req_id);

        // write context meta on loaned shm
        buf_oper.SetUint8(static_cast<uint8_t>(keys.size()));
        for (const auto& s : context_meta_kv) {
          buf_oper.SetString(s, util::BufferLenType::kUInt16);
        }

        header_len = 1 + serialization_type.size() +
                     1 + pattern.size() +
                     4 +
                     context_meta_kv_size;

        // write msg on loaned shm： should start at the (FIXED_LEN + header_len)-th byte
        aimrt::util::ZenohBufferArrayAllocator z_allocator(buf_oper.GetRemainingSize(), z_pub_loaned_shm_ptr + header_len + 4);

        if (buffer_array_cache_ptr == nullptr) {
          try {
            auto result = SerializeReqSupportedZenoh(*client_invoke_wrapper_ptr, serialization_type, aimrt::util::BufferArrayAllocatorRef(z_allocator.NativeHandle()));
            msg_size = result.second;
            buffer_array_cache_ptr = result.first;
            if (buffer_array_cache_ptr == nullptr) {
              // in this case means no cache is set, then do nomal serialization(if size is small will throw exception)
              is_shm_loan_size_enough = true;
            } else {
              if (msg_size > buf_oper.GetRemainingSize()) {
                // in this case means the msg has serialization cache but the size is too large, then expand suitable size
                is_shm_loan_size_enough = false;
                {
                  std::unique_lock lock(z_shared_mutex_);
                  z_node_shm_size_map_[node_pub_topic] = 4 + header_len + msg_size;
                }
              } else {
                // in this case means the msg has serialization cache and the size is suitable, then use cachema
                is_shm_loan_size_enough = true;
              }
            }

          } catch (const std::exception& e) {
            if (!z_allocator.IsShmEnough()) {
              // the shm is not enough, need to expand a double size
              {
                std::unique_lock lock(z_shared_mutex_);
                auto& ret = z_node_shm_size_map_[node_pub_topic];
                ret = ret << 1;
              }
              is_shm_loan_size_enough = false;
            } else {
              AIMRT_ERROR(
                  "Msg serialization failed, serialization_type {}, pkg_path: {}, module_name: {}, func_name: {}, exception: {}",
                  serialization_type, info.pkg_path, info.module_name, info.func_name, e.what());
              return;
            }
          }
        }
      } while (!is_shm_loan_size_enough);

      if (is_shm_pool_size_enough) {
        // if has cache, the copy it to shm to replace the serialization
        if (buffer_array_cache_ptr != nullptr) {
          unsigned char* strat_pos = z_pub_loaned_shm_ptr + 4 + header_len;
          for (size_t ii = 0; ii < buffer_array_cache_ptr->Size(); ++ii) {
            std::memcpy(strat_pos, buffer_array_cache_ptr.get()[ii].Data()->data, buffer_array_cache_ptr.get()[ii].Data()->len);
            strat_pos += buffer_array_cache_ptr.get()[ii].Data()->len;
          }

          buffer_array_cache_ptr = nullptr;
        }

        // write info pkg length on loaned shm
        util::SetBufFromUint32(reinterpret_cast<char*>(z_pub_loaned_shm_ptr), header_len);

        z_owned_bytes_t z_payload;
        if (loan_result.status == ZC_BUF_LAYOUT_ALLOC_STATUS_OK) {
          z_bytes_from_shm_mut(&z_payload, z_move(loan_result.buf));
        }
        z_publisher_put(z_loan(z_pub_ctx_ptr->z_pub), z_move(z_payload), &zenoh_manager_ptr_->z_pub_options_);

        // collect garbage and defragment shared memory, whose reference counting is zero
        z_shm_provider_garbage_collect(z_loan(zenoh_manager_ptr_->shm_provider_));
        z_shm_provider_defragment(z_loan(zenoh_manager_ptr_->shm_provider_));

        AIMRT_TRACE("Zenoh Invoke req  with '{}'", pattern);
        return;
      }
    }

    // shm disabled
    // client req serialize
    auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeReqWithCache(
        *client_invoke_wrapper_ptr, serialization_type);
    if (!buffer_array_view_ptr) [[unlikely]] {
      // serialize failed
      client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED));
      return;
    }

    const auto* buffer_array_data = buffer_array_view_ptr->Data();
    const size_t buffer_array_len = buffer_array_view_ptr->Size();
    size_t req_size = buffer_array_view_ptr->BufferSize();

    // padding zenoh pkg (serialization_type + pattern +  req_id + context_meta + req)
    size_t z_data_size = 1 + serialization_type.size() +
                         1 + pattern.size() +
                         4 +
                         context_meta_kv_size +
                         req_size;

    size_t pkg_size = z_data_size + 4;
    // create buffer for serialization
    std::vector<char> msg_buf_vec(pkg_size);
    util::BufferOperator buf_oper(msg_buf_vec.data(), pkg_size);

    // full data_size
    buf_oper.SetUint32(z_data_size);

    // full serialization_type
    buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

    // full pattern
    buf_oper.SetString(pattern, util::BufferLenType::kUInt8);

    // full req id
    buf_oper.SetUint32(cur_req_id);

    // full context meta
    buf_oper.SetUint8(static_cast<uint8_t>(keys.size()));
    for (const auto& s : context_meta_kv) {
      buf_oper.SetString(s, util::BufferLenType::kUInt16);
    }

    // full client req
    for (size_t ii = 0; ii < buffer_array_len; ++ii) {
      buf_oper.SetBuffer(
          static_cast<const char*>(buffer_array_data[ii].data),
          buffer_array_data[ii].len);
    }

    // send req
    z_owned_bytes_t z_payload;
    z_bytes_from_buf(&z_payload, reinterpret_cast<uint8_t*>(msg_buf_vec.data()), pkg_size, nullptr, nullptr);
    z_publisher_put(z_loan(z_pub_ctx_ptr->z_pub), z_move(z_payload), &zenoh_manager_ptr_->z_pub_options_);
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void ZenohRpcBackend::RegisterGetExecutorFunc(
    const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::PreInit,
      "Method can only be called when state is 'PreInit'.");
  get_executor_func_ = get_executor_func;
}

void ZenohRpcBackend::ReturnRspWithStatusCode(
    const std::string& pattern,
    std::string_view serialization_type,
    const char* req_id_buf,
    uint32_t code) {
  namespace util = aimrt::common::util;

  int pkg_size = 1 + serialization_type.size() + 4 + 4;
  std::vector<char> msg_buf_vec(pkg_size);
  util::BufferOperator buf_oper(msg_buf_vec.data(), msg_buf_vec.size());

  buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);
  buf_oper.SetBuffer(req_id_buf, 4);
  buf_oper.SetUint32(code);

  zenoh_manager_ptr_->Publish("rsp/" + pattern, msg_buf_vec.data(), pkg_size);
}

}  // namespace aimrt::plugins::zenoh_plugin