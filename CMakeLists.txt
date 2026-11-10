# carla-expert-nmpc: the NMPC and its planner (nmpc/), the middle layer's perception (middle_layer/), their offline
# closed-loop scenarios (sim/urban), and the Python module expertctl the Bench2Drive agents use.
#
#   cmake -S . -B build && cmake --build build -j && ctest --test-dir build
#
# Python (for CasADi and pybind11) is SimLingo's venv, external/simlingo_venv, since the agents run in the
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

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)  # the planner solves its candidates in parallel
find_package(nlohmann_json 3.2 QUIET)  # vehicle parameter files
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
    FetchContent_MakeAvailable(json)
endif()

get_target_property(CASADI_LIBRARY casadi::casadi IMPORTED_LOCATION_RELEASE)
get_filename_component(CASADI_LIB_DIR ${CASADI_LIBRARY} DIRECTORY)
set(RPATH ${CASADI_LIB_DIR})
enable_testing()

# ---- the two layers: libraries nmpc_shared (paths, vehicle parameters), perception and nmpc

add_subdirectory(nmpc)
add_subdirectory(middle_layer)

add_executable(urban_scenarios sim/urban/scenarios.cpp)  # offline closed-loop scenarios for the urban NMPC
target_link_libraries(urban_scenarios PRIVATE nmpc)
target_compile_options(urban_scenarios PRIVATE -Wno-missing-field-initializers)  # scenarios are aggregates, partly filled
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/urban_cache)
target_compile_definitions(urban_scenarios PRIVATE URBAN_CACHE_DIR="${CMAKE_BINARY_DIR}/urban_cache"
    SIM_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
set_target_properties(urban_scenarios PROPERTIES BUILD_RPATH "${RPATH}")
add_test(NAME urban.scenarios COMMAND urban_scenarios)

# ---- Python module for the agents: build/python/expertctl*.so

pybind11_add_module(expertctl bindings/expertctl.cpp)
target_link_libraries(expertctl PRIVATE nmpc)
set_target_properties(expertctl PROPERTIES BUILD_RPATH "${RPATH}" LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)
