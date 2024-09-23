// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/rpc/rpc_registry.h"

namespace aimrt::runtime::core::rpc {

bool RpcRegistry::RegisterServiceFunc(
    std::unique_ptr<ServiceFuncWrapper>&& service_func_wrapper_ptr) {
  Key key{
      .func_name = service_func_wrapper_ptr->info.func_name,
      .pkg_path = service_func_wrapper_ptr->info.pkg_path,
      .module_name = service_func_wrapper_ptr->info.module_name};

  auto emplace_ret = service_func_wrapper_map_.emplace(
      key, std::move(service_func_wrapper_ptr));

  if (!emplace_ret.second) {
    AIMRT_ERROR(
        "Service func '{}' is registered repeatedly, module '{}', pkg path '{}'",
        key.func_name, key.module_name, key.pkg_path);
    return false;
  }

  service_index_map_[key.func_name].emplace_back(emplace_ret.first->second.get());

  AIMRT_TRACE(
      "Service func '{}' is successfully registered, module '{}', pkg path '{}'",
      key.func_name, key.module_name, key.pkg_path);

  return true;
}

bool RpcRegistry::RegisterClientFunc(
    std::unique_ptr<ClientFuncWrapper>&& client_func_wrapper_ptr) {
  Key key{
      .func_name = client_func_wrapper_ptr->info.func_name,
      .pkg_path = client_func_wrapper_ptr->info.pkg_path,
      .module_name = client_func_wrapper_ptr->info.module_name};

  auto emplace_ret = client_func_wrapper_map_.emplace(
      key, std::move(client_func_wrapper_ptr));

  if (!emplace_ret.second) {
    AIMRT_ERROR(
        "Client func '{}' is registered repeatedly, module '{}', pkg path '{}'",
        key.func_name, key.module_name, key.pkg_path);
    return false;
  }

  client_index_map_[key.func_name].emplace_back(emplace_ret.first->second.get());

  AIMRT_TRACE(
      "Client func '{}' is successfully registered, module '{}', pkg path '{}'",
      key.func_name, key.module_name, key.pkg_path);

  return true;
}

const ServiceFuncWrapper* RpcRegistry::GetServiceFuncWrapperPtr(
    std::string_view func_name, std::string_view pkg_path, std::string_view module_name) const {
  auto find_itr = service_func_wrapper_map_.find(
      Key{.func_name = func_name, .pkg_path = pkg_path, .module_name = module_name});

  if (find_itr != service_func_wrapper_map_.end())
    return find_itr->second.get();

  return nullptr;
}

const ClientFuncWrapper* RpcRegistry::GetClientFuncWrapperPtr(
    std::string_view func_name, std::string_view pkg_path, std::string_view module_name) const {
  auto find_itr = client_func_wrapper_map_.find(
      Key{.func_name = func_name, .pkg_path = pkg_path, .module_name = module_name});

  if (find_itr != client_func_wrapper_map_.end())
    return find_itr->second.get();

  return nullptr;
}

}  // namespace aimrt::runtime::core::rpc
