function(check_python_package package_name result_var)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pip show ${package_name}
    RESULT_VARIABLE result
    OUTPUT_QUIET ERROR_QUIET)
  if(NOT ${result} EQUAL 0)
    set(${result_var}
        OFF
        PARENT_SCOPE)
    message(WARNING "Cannot find ${package_name} in your Python environment!")
  else()
    set(${result_var}
        ON
        PARENT_SCOPE)
  endif()
endfunction()
