cmake_minimum_required (VERSION 3.1)

# cmake_policy(PUSH)
# cmake_policy(POP)
# cmake_policy(SET CMP0048 OLD)
project (embreeUnityPlugin)
#set(PROJECT_VERSION_MAJOR "1")
#set(PROJECT_VERSION_MINOR "0")

## third party libraries
set(EMBREE_TASKING_SYSTEM "INTERNAL" CACHE STRING "" FORCE)
set(EMBREE_ISPC_SUPPORT OFF CACHE BOOL "" FORCE)
# option(GLFW_BUILD_DOCS OFF)
add_subdirectory(thirdParty/embree)
include_directories (thirdParty/embree/include EXCLUDE_FROM_ALL)

## set compiler flags
set (CMAKE_CXX_STANDARD 11)
set (CXX_STANDARD_REQUIRED ON)
# set (CMAKE_CXX_EXTENSIONS OFF)

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # set (warnings "-Wall -Wextra -Wpedantic -Werror")
    # set (warnings "-Wall -Wextra")  # had to turn off pedantic in order to compile glad
    set (warnings "-Wall -Wextra -Werror")  # had to turn off pedantic in order to compile glad
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC") #if (MSVC)
    set (warnings "/W4 /WX /EHsc")
endif()

set (CMAKE_C_FLAGS ${CMAKE_C_FLAGS} ${warnings})
set (CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${warnings})

# if (NOT configuredOnce)
#     set (CMAKE_C_FLAGS ${warnings} CACHE STRING "flags used by the compiler during all build types." FORCE)
#     set (CMAKE_CXX_FLAGS ${warnings} CACHE STRING "flags used by the compiler during all build types." FORCE)
# endif ()
## end compiler flags


## own project
include_directories (include)
set (PROJECT_SOURCES src/main.cpp)
set (PROJECT_LIBRARIES embree)

add_executable (${PROJECT_NAME} ${PROJECT_SOURCES})

target_link_libraries (${PROJECT_NAME} ${PROJECT_LIBRARIES})

# ## test library to use opengl within unity
# add_library (glReadPixels SHARED 
#     src/glReadPixels.cpp
#     ${THIRD_PARTY_SOURCES})
# target_link_libraries (glReadPixels glfw ${GLAD_LIBRARY_DEPENDENCIES})
# 
# enable_testing ()
# add_test (kalsengiTest ${PROJECT_NAME}) # add_test (testName target)

# set (configuredOnce TRUE CACHE INTERNAL "a flag showing that CMake has configured at least once.")
