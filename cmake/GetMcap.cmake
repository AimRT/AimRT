# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)
FetchContent_Declare(mcap_builder GIT_REPOSITORY https://github.com/olympus-robotics/mcap_builder.git GIT_TAG main)

FetchContent_MakeAvailable(mcap_builder)
function(get_mcap)

endfunction()

get_mcap()
