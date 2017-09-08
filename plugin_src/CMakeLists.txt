cmake_minimum_required (VERSION 3.1)

project (embreeUnityPlugin)

## own project
add_library (${PROJECT_NAME} SHARED voxel_visibility_checker.cpp)
target_link_libraries (embreeUnityPlugin embree)
