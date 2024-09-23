# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

macro(set_root_namespace arg1)
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY __CURRENT_NAMESPACE__ ${arg1})
endmacro()

macro(set_namespace)
  string(REGEX REPLACE ".*/\(.*\)" "\\1" __CUR_DIR__ ${CMAKE_CURRENT_SOURCE_DIR})
  get_directory_property(__CUR_DIRECTORY_PARENT__ PARENT_DIRECTORY)
  get_property(
    __SUPERIOR_NAMESPACE__
    DIRECTORY ${__CUR_DIRECTORY_PARENT__}
    PROPERTY __CURRENT_NAMESPACE__)
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY __CURRENT_NAMESPACE__ ${__SUPERIOR_NAMESPACE__}::${__CUR_DIR__})
endmacro()

macro(get_namespace arg1)
  get_directory_property(__CUR_DIRECTORY_PARENT__ PARENT_DIRECTORY)
  get_property(
    ${arg1}
    DIRECTORY ${__CUR_DIRECTORY_PARENT__}
    PROPERTY __CURRENT_NAMESPACE__)
  if(${arg1} STREQUAL "")
    message(FATAL_ERROR "Can not get namespace for ${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
endmacro()
