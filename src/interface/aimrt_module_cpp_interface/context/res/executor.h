// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/context/res/details/base.h"

namespace aimrt::context::res {

/**
 * @brief 执行器资源描述符。
 *
 * 仅在模块初始化阶段创建，用于描述某个 AimRT 执行器在当前 Context 下的使用权限。
 */
class Executor : public details::Base {
 public:
  using details::Base::Base;
};

}  // namespace aimrt::context::res
