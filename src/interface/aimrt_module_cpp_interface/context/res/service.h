// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/context/res/details/base.h"

namespace aimrt::context::res {

template <class Q, class P>
class Service : public details::Base {
 public:
  using RequestType = Q;
  using ResponseType = P;
  using details::Base::Base;
};

}  // namespace aimrt::context::res


