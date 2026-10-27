# carla-expert-nmpc: the NMPC core (track_control and the urban NMPC to come), an offline bicycle plant for
# closed-loop tests, and the Python module expertctl the Bench2Drive agents use.
#
#   cmake -S . -B build && cmake --build build -j && ctest --test-dir build
#
# Python (for CasADi and pybind11) is SimLingo's venv, external/simlingo_venv, since both agents run in the
# leaderboard's process there, unless Python3_EXECUTABLE is given.
cmake_minimum_required(VERSION 3.18)
project(carla_expert_nmpc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
# no fused multiply-add, so results do not depend on the CPU the code was built for
add_compile_options(-Wall -Wextra -ffp-contract=off)
# CasADi's pip wheel uses the old libstdc++ string ABI; so must everything linked with it
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)

# ---- Python: CasADi's CMake package and pybind11 come from its site-packages

if(NOT Python3_EXECUTABLE AND EXISTS ${CMAKE_SOURCE_DIR}/external/simlingo_venv/bin/python)
    set(Python3_EXECUTABLE ${CMAKE_SOURCE_DIR}/external/simlingo_venv/bin/python)
endif()
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
if(NOT casadi_DIR)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import casadi, os; print(os.path.join(os.path.dirname(casadi.__file__), 'cmake'))"
        OUTPUT_VARIABLE casadi_DIR OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE casadi_missing)
    if(casadi_missing)
        message(FATAL_ERROR "CasADi not found in ${Python3_EXECUTABLE}: pip install casadi")
    endif()
endif()
execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(pybind11 CONFIG REQUIRED)
find_package(casadi CONFIG REQUIRED)

# ---- the library (core/track_control, copied from carla-track-control: PROVENANCE.md)

enable_testing()
set(TRACK_CONTROL_BUILD_TESTS ON)  # a normal variable overrides the subproject's option() (CMP0077)
set(TRACK_CONTROL_BUILD_EXAMPLES OFF)
add_subdirectory(core/track_control)
target_compile_definitions(test_model PRIVATE TEST_VEHICLE_FILE="${CMAKE_SOURCE_DIR}/config/vehicles/model3.json")

get_target_property(CASADI_LIBRARY casadi::casadi IMPORTED_LOCATION_RELEASE)
get_filename_component(CASADI_LIB_DIR ${CASADI_LIBRARY} DIRECTORY)
set(RPATH ${CASADI_LIB_DIR})

# ---- the offline plant, for closed-loop tests without CARLA

find_package(Threads REQUIRED)  # the bicycle plant's real-time thread
add_library(sim STATIC sim/plant/bicycle_plant.cpp)
target_include_directories(sim PUBLIC ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/core)
target_link_libraries(sim PUBLIC track_control::track_control Threads::Threads)
target_compile_definitions(sim PUBLIC SIM_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")

add_executable(test_bicycle_realtime sim/plant/test_bicycle_realtime.cpp)
target_link_libraries(test_bicycle_realtime PRIVATE sim)
set_target_properties(test_bicycle_realtime PROPERTIES BUILD_RPATH "${RPATH}")
add_test(NAME sim.test_bicycle_realtime COMMAND test_bicycle_realtime)

# ---- Python module for the agents: build/python/expertctl*.so

pybind11_add_module(expertctl bindings/expertctl.cpp)
target_link_libraries(expertctl PRIVATE sim)
set_target_properties(expertctl PROPERTIES BUILD_RPATH "${RPATH}" LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)
