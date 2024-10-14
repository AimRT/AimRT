# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

add_subdirectory(core)
add_subdirectory(main)

if(AIMRT_BUILD_PYTHON_RUNTIME)
  add_subdirectory(python_runtime)
endif()
