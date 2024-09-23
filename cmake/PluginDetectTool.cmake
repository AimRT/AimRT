# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the names of all subdirectories in a directory
function(get_subdirectories DIR OUT_VAR)
  file(GLOB ALL_FILES_AND_DIRS "${DIR}/*")

  set(SUBDIRS "")

  foreach(FILE_OR_DIR ${ALL_FILES_AND_DIRS})
    if(IS_DIRECTORY ${FILE_OR_DIR})
      get_filename_component(DIR_NAME ${FILE_OR_DIR} NAME)
      list(APPEND SUBDIRS ${DIR_NAME})
    endif()
  endforeach()

  set(${OUT_VAR}
      ${SUBDIRS}
      PARENT_SCOPE)
endfunction()
