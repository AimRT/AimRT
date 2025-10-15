// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstddef>
#include <string>

namespace aimrt::context {
class Context;
class OpPub;
class OpSub;
}  // namespace aimrt::context

namespace aimrt::context::res::details {

/**
 * @brief 资源标识基础类，保存名称与所隶属的 Context 信息。
 */
class Base {
 public:
  Base() = default;

  explicit Base(std::string name) : name_(std::move(name)) {}

  [[nodiscard]] bool IsValid() const {
    return idx_ != static_cast<std::size_t>(-1) && context_id_ != -1;
  }

  [[nodiscard]] const std::string& GetName() const { return name_; }

 protected:
  static void SetName(Base& obj, std::string name) { obj.name_ = std::move(name); }

 private:
  friend class aimrt::context::Context;
  friend class aimrt::context::OpPub;
  friend class aimrt::context::OpSub;

  std::string name_;
  std::size_t idx_ = static_cast<std::size_t>(-1);
  int context_id_ = -1;
};

}  // namespace aimrt::context::res::details
