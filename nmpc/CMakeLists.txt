# The NMPC: the optimisation (framework, cost, the manoeuvres' constraints in constraint/) and the planner that runs
# several of them. nmpc_shared is what the middle layer needs of it too: paths and the vehicle's parameters.

add_library(nmpc_shared STATIC
    src/path.cpp
    src/vehicle_params.cpp)
target_include_directories(nmpc_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nmpc_shared PUBLIC Eigen3::Eigen PRIVATE nlohmann_json::nlohmann_json)

add_library(nmpc STATIC
    src/framework.cpp
    src/cost.cpp
    src/planner.cpp
    src/constraint/candidates.cpp
    src/constraint/lane.cpp
    src/constraint/movers.cpp
    src/constraint/follow.cpp
    src/constraint/stop.cpp
    src/constraint/crossing.cpp
    src/constraint/overtake.cpp
    src/constraint/turn.cpp
    src/constraint/grip.cpp)
target_include_directories(nmpc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nmpc PUBLIC nmpc_shared perception Threads::Threads PRIVATE casadi::casadi)

add_executable(test_path tests/test_path.cpp)
target_link_libraries(test_path PRIVATE nmpc_shared)
set_target_properties(test_path PROPERTIES BUILD_RPATH "${RPATH}")
add_test(NAME nmpc.test_path COMMAND test_path)
