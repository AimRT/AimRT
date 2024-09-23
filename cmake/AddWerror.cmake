# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

macro(add_werror target)
  if(AIMRT_BUILD_WITH_WERROR)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(
        ${target}
        PRIVATE -Wall
                -Wextra
                -Wno-unused-parameter
                -Wno-missing-field-initializers
                -Wno-implicit-fallthrough
                -Wno-stringop-overflow
                -Werror)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
      target_compile_options(${target} PRIVATE /W4 /WX)
    endif()
  endif()
endmacro()
