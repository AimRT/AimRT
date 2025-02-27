// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <list>

#include "core/rpc/rpc_registry.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::rpc {

class RpcBackendBase {
 public:
  RpcBackendBase() = default;
  virtual ~RpcBackendBase() = default;

  RpcBackendBase(const RpcBackendBase&) = delete;
  RpcBackendBase& operator=(const RpcBackendBase&) = delete;

  virtual std::string_view Name() const noexcept = 0;  // It should always return the same value

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual void Start() = 0;
  virtual void Shutdown() = 0;

  virtual std::list<std::pair<std::string, std::string>> GenInitializationReport() const noexcept { return {}; }

  /**
   * @brief Set the Rpc Registry to backend
   * @note
   * 1. This method will only be called once before 'Initialize'.
   *
   * @param rpc_registry_ptr
   */
  virtual void SetRpcRegistry(const RpcRegistry* rpc_registry_ptr) noexcept {}

  /**
   * @brief Register service func
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Start'.
   *
   * @param service_func_wrapper
   * @return Register result
   */
  virtual bool RegisterServiceFunc(
      const ServiceFuncWrapper& service_func_wrapper) noexcept = 0;

  /**
   * @brief Register client func
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Start'.
   *
   * @param client_func_wrapper
   * @return Register result
   */
  virtual bool RegisterClientFunc(
      const ClientFuncWrapper& client_func_wrapper) noexcept = 0;

  /**
   * @brief Invoke a rpc call
   * @note
   * 1. This method will only be called after 'Start' and before 'Shutdown'.
   *
   * @param client_invoke_wrapper_ptr
   */
  virtual void Invoke(
      const std::shared_ptr<InvokeWrapper>& client_invoke_wrapper_ptr) noexcept = 0;
};

}  // namespace aimrt::runtime::core::rpc
