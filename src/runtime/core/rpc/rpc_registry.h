// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "core/rpc/rpc_invoke_wrapper.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::rpc {

using ServiceFunc = std::function<void(const std::shared_ptr<InvokeWrapper>&)>;

struct ServiceFuncWrapper {
  FuncInfo info;
  ServiceFunc service_func;
};

struct ClientFuncWrapper {
  FuncInfo info;
};

class RpcRegistry {
 public:
  RpcRegistry()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~RpcRegistry() = default;

  RpcRegistry(const RpcRegistry&) = delete;
  RpcRegistry& operator=(const RpcRegistry&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  bool RegisterServiceFunc(
      std::unique_ptr<ServiceFuncWrapper>&& service_func_wrapper_ptr);

  bool RegisterClientFunc(
      std::unique_ptr<ClientFuncWrapper>&& client_func_wrapper_ptr);

  const ServiceFuncWrapper* GetServiceFuncWrapperPtr(
      std::string_view func_name, std::string_view pkg_path, std::string_view module_name) const;

  const ClientFuncWrapper* GetClientFuncWrapperPtr(
      std::string_view func_name, std::string_view pkg_path, std::string_view module_name) const;

  const auto& GetServiceFuncWrapperMap() const { return service_func_wrapper_map_; }
  const auto& GetClientFuncWrapperMap() const { return client_func_wrapper_map_; }

  const auto& GetServiceIndexMap() const { return service_index_map_; }
  const auto& GetClientIndexMap() const { return client_index_map_; }

 private:
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  struct Key {
    std::string_view func_name;
    std::string_view pkg_path;
    std::string_view module_name;

    bool operator==(const Key& rhs) const {
      return func_name == rhs.func_name &&
             pkg_path == rhs.pkg_path &&
             module_name == rhs.module_name;
    }

    struct Hash {
      std::size_t operator()(const Key& k) const {
        return (std::hash<std::string_view>()(k.func_name)) ^
               (std::hash<std::string_view>()(k.pkg_path)) ^
               (std::hash<std::string_view>()(k.module_name));
      }
    };
  };

  std::unordered_map<Key, std::unique_ptr<ServiceFuncWrapper>, Key::Hash>
      service_func_wrapper_map_;

  // 索引表，func_name:wrapper
  std::unordered_map<std::string_view, std::vector<ServiceFuncWrapper*>>
      service_index_map_;

  std::unordered_map<Key, std::unique_ptr<ClientFuncWrapper>, Key::Hash>
      client_func_wrapper_map_;

  // 索引表，func_name:wrapper
  std::unordered_map<std::string_view, std::vector<ClientFuncWrapper*>>
      client_index_map_;
};
}  // namespace aimrt::runtime::core::rpc
