// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <memory>
#include <memory_resource>
#include <string_view>
#include <unordered_map>

#include "aimrt_module_c_interface/rpc/rpc_context_base.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "util/exception.h"
#include "util/string_util.h"

namespace aimrt::rpc {

class Context {
 public:
  explicit Context(aimrt_rpc_context_type_t type = aimrt_rpc_context_type_t::AIMRT_RPC_CLIENT_CONTEXT)
      : meta_data_map_(&default_pool_),
        meta_keys_vec_(&default_pool_),
        type_(type),
        base_(aimrt_rpc_context_base_t{
            .ops = GenOpsBase(),
            .impl = this}) {}
  ~Context() = default;

  Context(const Context& other)
      : used_(other.used_),
        timeout_ns_(other.timeout_ns_),
        meta_data_map_(&default_pool_),
        meta_keys_vec_(&default_pool_),
        type_(other.type_),
        base_(aimrt_rpc_context_base_t{
            .ops = GenOpsBase(),
            .impl = this}) {}

  const aimrt_rpc_context_base_t* NativeHandle() const { return &base_; }

  bool CheckUsed() const { return used_; }
  void SetUsed() { used_ = true; }

  void Reset() {
    used_ = false;
    timeout_ns_ = 0;

    meta_data_map_.clear();
    meta_keys_vec_.clear();
  }

  aimrt_rpc_context_type_t GetType() const {
    return type_;
  }

  // Timeout
  std::chrono::nanoseconds Timeout() const {
    return std::chrono::nanoseconds(timeout_ns_);
  }

  void SetTimeout(std::chrono::nanoseconds timeout) {
    timeout_ns_ = timeout.count();
  }

  // Some frame fields
  std::string_view GetMetaValue(std::string_view key) const {
    auto finditr = meta_data_map_.find(key);
    if (finditr != meta_data_map_.end()) return finditr->second;
    return "";
  }

  void SetMetaValue(std::string_view key, std::string_view val) {
    auto finditr = meta_data_map_.find(key);
    if (finditr == meta_data_map_.end()) {
      meta_data_map_.emplace(key, val);
    } else {
      finditr->second = std::string(val);
    }
  }

  std::vector<std::string_view> GetMetaKeys() const {
    std::vector<std::string_view> result;
    result.reserve(meta_data_map_.size());
    for (const auto& it : meta_data_map_) result.emplace_back(it.first);
    return result;
  }

  std::string_view GetToAddr() const {
    return GetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR);
  }
  void SetToAddr(std::string_view val) {
    SetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR, val);
  }

  std::string_view GetSerializationType() const {
    return GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE);
  }
  void SetSerializationType(std::string_view val) {
    SetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE, val);
  }

  std::string_view GetFunctionName() const {
    return GetMetaValue(AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME);
  }
  void SetFunctionName(std::string_view val) {
    SetMetaValue(AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME, val);
  }

  std::string ToString() const {
    std::stringstream ss;
    if (type_ == aimrt_rpc_context_type_t::AIMRT_RPC_CLIENT_CONTEXT) {
      ss << "Client context, ";
    } else if (type_ == aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT) {
      ss << "Server context, ";
    } else {
      ss << "Unknown context, ";
    }

    ss << "timeout: " << timeout_ns_ / 1000000 << "ms, meta: {";

    for (auto itr = meta_data_map_.begin(); itr != meta_data_map_.end(); ++itr) {
      if (itr != meta_data_map_.begin()) ss << ",";
      ss << "{\"" << itr->first << "\":\"" << itr->second << "\"}";
    }

    ss << "}";

    return ss.str();
  }

 private:
  static const aimrt_rpc_context_base_ops_t* GenOpsBase() {
    static constexpr aimrt_rpc_context_base_ops_t kOps{
        .check_used = [](void* impl) -> bool {
          return static_cast<Context*>(impl)->used_;
        },
        .set_used = [](void* impl) {
          static_cast<Context*>(impl)->used_ = true;  //
        },
        .get_type = [](void* impl) -> aimrt_rpc_context_type_t {
          return static_cast<Context*>(impl)->type_;
        },
        .get_timeout_ns = [](void* impl) -> uint64_t {
          return static_cast<Context*>(impl)->timeout_ns_;
        },
        .set_timeout_ns = [](void* impl, uint64_t timeout) {
          static_cast<Context*>(impl)->timeout_ns_ = timeout;  //
        },
        .get_meta_val = [](void* impl, aimrt_string_view_t key) -> aimrt_string_view_t {
          return aimrt::util::ToAimRTStringView(
              static_cast<Context*>(impl)->GetMetaValue(aimrt::util::ToStdStringView(key)));
        },
        .set_meta_val = [](void* impl, aimrt_string_view_t key, aimrt_string_view_t val) {
          static_cast<Context*>(impl)->SetMetaValue(
              aimrt::util::ToStdStringView(key), aimrt::util::ToStdStringView(val));  //
        },
        .get_meta_keys = [](void* impl) -> aimrt_string_view_array_t {
          const auto& meta_data_map = static_cast<Context*>(impl)->meta_data_map_;
          auto& meta_keys_vec = static_cast<Context*>(impl)->meta_keys_vec_;

          meta_keys_vec.clear();
          meta_keys_vec.reserve(meta_data_map.size());

          for (const auto& it : meta_data_map)
            meta_keys_vec.emplace_back(aimrt::util::ToAimRTStringView(it.first));

          return aimrt_string_view_array_t{
              .str_array = meta_keys_vec.data(),
              .len = meta_keys_vec.size()};
        }};

    return &kOps;
  }

 private:
  bool used_ = false;

  uint64_t timeout_ns_ = 0;

  alignas(std::max_align_t) std::array<std::byte, 512> buffer_;
  std::pmr::monotonic_buffer_resource default_pool_{buffer_.data(), buffer_.size()};

  std::pmr::unordered_map<
      std::pmr::string,
      std::pmr::string,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      meta_data_map_;

  std::pmr::vector<aimrt_string_view_t> meta_keys_vec_;

  const aimrt_rpc_context_type_t type_;
  const aimrt_rpc_context_base_t base_;
};

class ContextRef {
 public:
  ContextRef() = default;
  ContextRef(const Context& ctx)
      : base_ptr_(ctx.NativeHandle()) {}
  ContextRef(const Context* ctx_ptr)
      : base_ptr_(ctx_ptr ? ctx_ptr->NativeHandle() : nullptr) {}
  ContextRef(const std::shared_ptr<Context>& ctx_ptr)
      : base_ptr_(ctx_ptr ? ctx_ptr->NativeHandle() : nullptr) {}
  explicit ContextRef(const aimrt_rpc_context_base_t* base_ptr)
      : base_ptr_(base_ptr) {}
  ~ContextRef() = default;

  explicit operator bool() const { return (base_ptr_ != nullptr); }

  const aimrt_rpc_context_base_t* NativeHandle() const {
    return base_ptr_;
  }

  bool CheckUsed() const {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");
    return base_ptr_->ops->check_used(base_ptr_->impl);
  }

  void SetUsed() {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");
    base_ptr_->ops->set_used(base_ptr_->impl);
  }

  aimrt_rpc_context_type_t GetType() const {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");
    return base_ptr_->ops->get_type(base_ptr_->impl);
  }

  // Timeout manager
  std::chrono::nanoseconds Timeout() const {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");
    return std::chrono::nanoseconds(base_ptr_->ops->get_timeout_ns(base_ptr_->impl));
  }

  void SetTimeout(std::chrono::nanoseconds timeout) {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");
    base_ptr_->ops->set_timeout_ns(base_ptr_->impl, timeout.count());
  }

  // Some frame fields
  std::string_view GetMetaValue(std::string_view key) const {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");
    return aimrt::util::ToStdStringView(
        base_ptr_->ops->get_meta_val(base_ptr_->impl, aimrt::util::ToAimRTStringView(key)));
  }

  void SetMetaValue(std::string_view key, std::string_view val) {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");
    base_ptr_->ops->set_meta_val(
        base_ptr_->impl, aimrt::util::ToAimRTStringView(key), aimrt::util::ToAimRTStringView(val));
  }

  std::vector<std::string_view> GetMetaKeys() const {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");
    aimrt_string_view_array_t keys = base_ptr_->ops->get_meta_keys(base_ptr_->impl);

    std::vector<std::string_view> result;
    result.reserve(keys.len);
    for (size_t ii = 0; ii < keys.len; ++ii)
      result.emplace_back(aimrt::util::ToStdStringView(keys.str_array[ii]));

    return result;
  }

  std::string_view GetToAddr() const {
    return GetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR);
  }
  void SetToAddr(std::string_view val) {
    SetMetaValue(AIMRT_RPC_CONTEXT_KEY_TO_ADDR, val);
  }

  std::string_view GetSerializationType() const {
    return GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE);
  }
  void SetSerializationType(std::string_view val) {
    SetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE, val);
  }

  std::string_view GetFunctionName() const {
    return GetMetaValue(AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME);
  }
  void SetFunctionName(std::string_view val) {
    SetMetaValue(AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME, val);
  }

  std::string ToString() const {
    AIMRT_ASSERT(base_ptr_ && base_ptr_->ops, "Reference is null.");

    std::stringstream ss;

    auto type = base_ptr_->ops->get_type(base_ptr_->impl);
    if (type == aimrt_rpc_context_type_t::AIMRT_RPC_CLIENT_CONTEXT) {
      ss << "Client context, ";
    } else if (type == aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT) {
      ss << "Server context, ";
    } else {
      ss << "Unknown context, ";
    }

    auto timeout = base_ptr_->ops->get_timeout_ns(base_ptr_->impl);
    ss << "timeout: " << timeout / 1000000 << "ms, meta: {";

    aimrt_string_view_array_t keys = base_ptr_->ops->get_meta_keys(base_ptr_->impl);
    for (size_t ii = 0; ii < keys.len; ++ii) {
      if (ii != 0) ss << ",";
      auto val = base_ptr_->ops->get_meta_val(base_ptr_->impl, keys.str_array[ii]);
      ss << "{\"" << aimrt::util::ToStdStringView(keys.str_array[ii])
         << "\":\"" << aimrt::util::ToStdStringView(val) << "\"}";
    }

    ss << "}";

    return ss.str();
  }

 private:
  const aimrt_rpc_context_base_t* base_ptr_;
};

}  // namespace aimrt::rpc