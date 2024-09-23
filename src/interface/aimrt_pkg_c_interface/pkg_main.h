// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <stddef.h>

#include "aimrt_module_c_interface/module_base.h"
#include "aimrt_pkg_c_interface/pkg_export.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief get length of the module name list
 *
 * @return length of the module name list
 */
AIMRT_PKG_EXPORT size_t AimRTDynlibGetModuleNum();

/**
 * @brief get module name list
 *
 * @return module name list
 */
AIMRT_PKG_EXPORT const aimrt_string_view_t* AimRTDynlibGetModuleNameList();

/**
 * @brief create module
 *
 * @param module_name must in the module name list
 * @return module
 */
AIMRT_PKG_EXPORT const aimrt_module_base_t* AimRTDynlibCreateModule(aimrt_string_view_t module_name);

/**
 * @brief destroy module
 *
 * @param module_ptr module created by 'AimRTDynlibCreateModule'
 */
AIMRT_PKG_EXPORT void AimRTDynlibDestroyModule(const aimrt_module_base_t* module_ptr);

#ifdef __cplusplus
}
#endif
