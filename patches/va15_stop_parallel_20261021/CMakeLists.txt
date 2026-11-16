cmake_minimum_required(VERSION 3.16)
project(va15_parallel_getdown_simulation_only LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
set(VA15_BASELINE_ROOT "/home/dong/rl_sar_b2_z1" CACHE PATH "Read-only framework")
set(VA15_TORCH_ROOT "/home/dong/rl_sar_b2_z1_old/library/inference_runtime/libtorch" CACHE PATH "Existing CPU LibTorch")
set(VA15_CORE_ARCHIVE "${CMAKE_CURRENT_SOURCE_DIR}/../va15_stop_feedback_20261020/work/build/libva15_core.a" CACHE FILEPATH "Unchanged previously compiled actor/core")
set(MUJOCO_ROOT "/home/dong/anaconda3/envs/crack_vision/lib/python3.8/site-packages/mujoco" CACHE PATH "Existing headless MuJoCo")
find_package(Torch REQUIRED PATHS "${VA15_TORCH_ROOT}/share/cmake/Torch" NO_DEFAULT_PATH)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(VA15_YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(VA15_YAML_TARGET yaml-cpp)
endif()
add_library(simulation_dependencies INTERFACE)
target_include_directories(simulation_dependencies INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../va15_independent_state_20261020"
  "${CMAKE_CURRENT_SOURCE_DIR}/../va15_stop_feedback_20261020/work")
foreach(DIR observation_buffer rl_sdk loop fsm vector_math motion_loader inference_runtime logger)
  target_include_directories(simulation_dependencies INTERFACE "${VA15_BASELINE_ROOT}/src/rl_sar/library/core/${DIR}")
endforeach()
target_compile_definitions(simulation_dependencies INTERFACE USE_CMAKE USE_TORCH
  VA15_ESTIMATED_STOP_SIMULATION_ONLY=1
  VA15_PARALLEL_GETDOWN_SIMULATION_ONLY=1
  POLICY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../va15_stop_feedback_20261020/work")
target_compile_options(simulation_dependencies INTERFACE -UNDEBUG)
target_link_options(simulation_dependencies INTERFACE -Wl,--disable-new-dtags)
target_link_libraries(simulation_dependencies INTERFACE Eigen3::Eigen ${VA15_YAML_TARGET} Threads::Threads
  "${VA15_CORE_ARCHIVE}" ${TORCH_LIBRARIES} tbb)
add_executable(test_simulation_feedback test_simulation_feedback.cpp)
target_link_libraries(test_simulation_feedback PRIVATE simulation_dependencies)
target_link_libraries(test_simulation_feedback PRIVATE -Wl,--no-as-needed
  "${VA15_TORCH_ROOT}/lib/libc10.so" -Wl,--as-needed)
add_test(NAME simulation_bridge_boundaries COMMAND test_simulation_feedback)
add_executable(test_b_stop_brake test_b_stop_brake.cpp)
target_compile_definitions(test_b_stop_brake PRIVATE VA15_ESTIMATED_STOP_SIMULATION_ONLY=1)
target_compile_options(test_b_stop_brake PRIVATE -UNDEBUG)
add_test(NAME b_only_command_brake COMMAND test_b_stop_brake)
add_executable(test_stop_response test_stop_response.cpp)
target_link_libraries(test_stop_response PRIVATE simulation_dependencies -Wl,--no-as-needed
  "${VA15_TORCH_ROOT}/lib/libc10.so" -Wl,--as-needed)
set_target_properties(test_stop_response PROPERTIES BUILD_RPATH "${VA15_TORCH_ROOT}/lib")
add_test(NAME unchanged_support_and_torque_continuity COMMAND test_stop_response)
add_executable(test_parallel_getdown test_parallel_getdown.cpp)
target_link_libraries(test_parallel_getdown PRIVATE simulation_dependencies -Wl,--no-as-needed
  "${VA15_TORCH_ROOT}/lib/libc10.so" -Wl,--as-needed)
set_target_properties(test_parallel_getdown PROPERTIES BUILD_RPATH "${VA15_TORCH_ROOT}/lib")
add_test(NAME parallel_getdown_boundaries COMMAND test_parallel_getdown)
set_tests_properties(simulation_bridge_boundaries PROPERTIES ENVIRONMENT
  "LD_LIBRARY_PATH=${VA15_TORCH_ROOT}/lib")
add_executable(simulate_independent_stop simulate_independent_stop.cpp)
target_include_directories(simulate_independent_stop PRIVATE "${MUJOCO_ROOT}/include")
target_link_libraries(simulate_independent_stop PRIVATE simulation_dependencies "${MUJOCO_ROOT}/libmujoco.so.3.2.3")
set_target_properties(test_simulation_feedback simulate_independent_stop PROPERTIES
  BUILD_RPATH "${VA15_TORCH_ROOT}/lib;${MUJOCO_ROOT}")
