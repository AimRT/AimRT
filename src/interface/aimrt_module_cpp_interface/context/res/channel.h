// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/context/res/details/base.h"

namespace aimrt::context::res {

template <class T>
class Channel : public details::Base {
 public:
  using MessageType = T;
  using details::Base::Base;
};

}  // namespace aimrt::context::res
