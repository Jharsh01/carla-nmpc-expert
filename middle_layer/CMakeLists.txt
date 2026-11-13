# The middle layer's perception: the C++ post-processing of what the sensors report (perception/sensors, Python,
# reading CARLA) into the route's path coordinates, for the NMPC (nmpc/).

add_library(perception STATIC
    perception/post_process/perception.cpp
    perception/post_process/tracks.cpp
    perception/post_process/classify.cpp
    perception/post_process/lanes.cpp
    perception/post_process/left_dir.cpp
    perception/post_process/right_dir.cpp)
target_include_directories(perception PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/perception/include)
target_link_libraries(perception PUBLIC nmpc_shared)
