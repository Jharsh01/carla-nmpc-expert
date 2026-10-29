# carla-expert-nmpc: the urban NMPC and its behaviour layer (core/urban), its offline closed-loop scenarios
# (sim/urban), and the Python module expertctl the Bench2Drive agents use.
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
find_package(Threads REQUIRED)  # the behaviour layer solves its candidates in parallel
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

# ---- the urban NMPC (core/urban)

add_library(urban STATIC
    core/urban/src/path.cpp
    core/urban/src/vehicle_params.cpp
    core/urban/src/urban_nmpc.cpp
    core/urban/src/behaviour.cpp)
target_include_directories(urban PUBLIC ${CMAKE_SOURCE_DIR}/core/urban/include)
target_link_libraries(urban PUBLIC Eigen3::Eigen Threads::Threads PRIVATE casadi::casadi nlohmann_json::nlohmann_json)

add_executable(test_path core/urban/tests/test_path.cpp)
target_link_libraries(test_path PRIVATE urban)
set_target_properties(test_path PROPERTIES BUILD_RPATH "${RPATH}")
add_test(NAME urban.test_path COMMAND test_path)

add_executable(urban_scenarios sim/urban/scenarios.cpp)  # offline closed-loop scenarios for the urban NMPC
target_link_libraries(urban_scenarios PRIVATE urban)
target_compile_options(urban_scenarios PRIVATE -Wno-missing-field-initializers)  # scenarios are aggregates, partly filled
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/urban_cache)
target_compile_definitions(urban_scenarios PRIVATE URBAN_CACHE_DIR="${CMAKE_BINARY_DIR}/urban_cache"
    SIM_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
set_target_properties(urban_scenarios PROPERTIES BUILD_RPATH "${RPATH}")
add_test(NAME urban.scenarios COMMAND urban_scenarios)

# ---- Python module for the agents: build/python/expertctl*.so

pybind11_add_module(expertctl bindings/expertctl.cpp)
target_link_libraries(expertctl PRIVATE urban)
set_target_properties(expertctl PROPERTIES BUILD_RPATH "${RPATH}" LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)
