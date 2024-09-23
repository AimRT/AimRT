// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/util/type_support_base.h"
#include "aimrt_type_support_pkg_c_interface/type_support_pkg_export.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief get length of the type support array
 *
 * @return length of the type support array
 */
AIMRT_TYPE_SUPPORT_PKG_EXPORT size_t AimRTDynlibGetTypeSupportArrayLength();

/**
 * @brief get the type support array
 *
 * @return type support array
 */
AIMRT_TYPE_SUPPORT_PKG_EXPORT const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray();

#ifdef __cplusplus
}
#endif
