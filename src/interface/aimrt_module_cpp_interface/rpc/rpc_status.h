// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "aimrt_module_c_interface/rpc/rpc_status_base.h"

namespace aimrt::rpc {

class Status {
 public:
  Status() = default;
  ~Status() = default;

  explicit Status(aimrt_rpc_status_code_t code) : code_(code) {}
  explicit Status(uint32_t code) : code_(code) {}

  bool OK() const {
    return code_ == aimrt_rpc_status_code_t::AIMRT_RPC_STATUS_OK;
  }

  operator bool() const { return OK(); }

  void SetCode(uint32_t code) { code_ = code; }
  void SetCode(aimrt_rpc_status_code_t code) { code_ = code; }

  uint32_t Code() const { return code_; }

  std::string ToString() const {
    return std::string(OK() ? "suc" : "fail") +
           ", code " + std::to_string(code_) +
           ", msg: " + std::string(GetCodeMsg(code_));
  }

  static std::string_view GetCodeMsg(uint32_t code) {
    static const std::unordered_map<uint32_t, std::string_view> kCodeMsgMap{
        {AIMRT_RPC_STATUS_OK, "OK"},
        {AIMRT_RPC_STATUS_UNKNOWN, "Unknown error"},
        {AIMRT_RPC_STATUS_TIMEOUT, "Timeout"},
        // svr side
        {AIMRT_RPC_STATUS_SVR_UNKNOWN, "Server side unknown error"},
        {AIMRT_RPC_STATUS_SVR_BACKEND_INTERNAL_ERROR, "Server side backend internal error"},
        {AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED, "Server not implemented"},
        {AIMRT_RPC_STATUS_SVR_NOT_FOUND, "Server not found"},
        {AIMRT_RPC_STATUS_SVR_INVALID_SERIALIZATION_TYPE, "Server side invalid serialization type"},
        {AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED, "Server side serialization failed"},
        {AIMRT_RPC_STATUS_SVR_INVALID_DESERIALIZATION_TYPE, "Server side invalid deserialization type"},
        {AIMRT_RPC_STATUS_SVR_DESERIALIZATION_FAILED, "Server side deserialization failed"},
        {AIMRT_RPC_STATUS_SVR_HANDLE_FAILED, "Server handle failed"},
        // cli side
        {AIMRT_RPC_STATUS_CLI_UNKNOWN, "Client side unknown error"},
        {AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR, "Client side backend internal error"},
        {AIMRT_RPC_STATUS_CLI_INVALID_CONTEXT, "Client side invalid context"},
        {AIMRT_RPC_STATUS_CLI_INVALID_ADDR, "Client side invalid address"},
        {AIMRT_RPC_STATUS_CLI_INVALID_SERIALIZATION_TYPE, "Client side invalid serialization type"},
        {AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED, "Client side serialization failed"},
        {AIMRT_RPC_STATUS_CLI_INVALID_DESERIALIZATION_TYPE, "Client side invalid deserialization type"},
        {AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED, "Client side deserialization failed"},
        {AIMRT_RPC_STATUS_CLI_NO_BACKEND_TO_HANDLE, "Client side no backend to handle"},
        {AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED, "Client side send req failed"},
        {AIMRT_RPC_STATUS_CLI_FUNC_NOT_REGISTERED, "Client side function not registered"}};

    auto finditr = kCodeMsgMap.find(code);
    return (finditr != kCodeMsgMap.end()) ? finditr->second : "Unknown code";
  }

 private:
  uint32_t code_ = aimrt_rpc_status_code_t::AIMRT_RPC_STATUS_OK;
};

}  // namespace aimrt::rpc
