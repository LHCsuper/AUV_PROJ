cmake_minimum_required(VERSION 3.20)

project(AUV_PROJ VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 QUIET)

set(AUV_LOCAL_EIGEN_DIR
    "${PROJECT_SOURCE_DIR}/tools/eigen/eigen-3.4.1"
)

add_library(auv_core STATIC
    src/actuator/saturation.cpp
    src/controller/path_following_controller.cpp
    src/environment/current_model.cpp
    src/guidance/los_guidance.cpp
    src/guidance/reference_path.cpp
    src/model/auv_3dof_model.cpp
    src/controller/pid_controller.cpp
    src/controller/speed_controller.cpp
    src/model/auv_3dof_params.cpp
    src/math/angle_utils.cpp
    src/model/heading_model.cpp
    src/math/rotation_2d.cpp
    src/simulator/logger_csv.cpp
)

target_include_directories(auv_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

if(Eigen3_FOUND)
    target_link_libraries(auv_core PUBLIC Eigen3::Eigen)
    target_compile_definitions(auv_core PUBLIC AUV_HAS_EIGEN=1)
elseif(EXISTS "${AUV_LOCAL_EIGEN_DIR}/Eigen/Dense")
    target_include_directories(auv_core PUBLIC "${AUV_LOCAL_EIGEN_DIR}")
    target_compile_definitions(auv_core PUBLIC AUV_HAS_EIGEN=1)
endif()

add_executable(auv_sim
    apps/auv_sim/main.cpp
)

target_link_libraries(auv_sim PRIVATE auv_core)

enable_testing()

add_executable(test_angle_utils
    tests/test_angle_utils.cpp
)
target_link_libraries(test_angle_utils PRIVATE auv_core)
add_test(NAME test_angle_utils COMMAND test_angle_utils)

add_executable(test_rotation_2d
    tests/test_rotation_2d.cpp
)
target_link_libraries(test_rotation_2d PRIVATE auv_core)
add_test(NAME test_rotation_2d COMMAND test_rotation_2d)

add_executable(test_csv_logger
    tests/test_csv_logger.cpp
)
target_link_libraries(test_csv_logger PRIVATE auv_core)
add_test(NAME test_csv_logger COMMAND test_csv_logger)

add_executable(test_pid_controller
    tests/test_pid_controller.cpp
)
target_link_libraries(test_pid_controller PRIVATE auv_core)
add_test(NAME test_pid_controller COMMAND test_pid_controller)

add_executable(test_heading_pid
    tests/test_heading_pid.cpp
)
target_link_libraries(test_heading_pid PRIVATE auv_core)
add_test(NAME test_heading_pid COMMAND test_heading_pid)

add_executable(test_rk4_integrator
    tests/test_rk4_integrator.cpp
)
target_link_libraries(test_rk4_integrator PRIVATE auv_core)
add_test(NAME test_rk4_integrator COMMAND test_rk4_integrator)

add_executable(test_auv_3dof_params
    tests/test_auv_3dof_params.cpp
)
target_link_libraries(test_auv_3dof_params PRIVATE auv_core)
add_test(NAME test_auv_3dof_params COMMAND test_auv_3dof_params)

add_executable(test_auv_3dof_model
    tests/test_auv_3dof_model.cpp
)
target_link_libraries(test_auv_3dof_model PRIVATE auv_core)
add_test(NAME test_auv_3dof_model COMMAND test_auv_3dof_model)

add_executable(test_speed_controller
    tests/test_speed_controller.cpp
)
target_link_libraries(test_speed_controller PRIVATE auv_core)
add_test(NAME test_speed_controller COMMAND test_speed_controller)

add_executable(test_actuator_saturation
    tests/test_actuator_saturation.cpp
)
target_link_libraries(test_actuator_saturation PRIVATE auv_core)
add_test(NAME test_actuator_saturation COMMAND test_actuator_saturation)

add_executable(test_current_model
    tests/test_current_model.cpp
)
target_link_libraries(test_current_model PRIVATE auv_core)
add_test(NAME test_current_model COMMAND test_current_model)

add_executable(test_los_guidance
    tests/test_los_guidance.cpp
)
target_link_libraries(test_los_guidance PRIVATE auv_core)
add_test(NAME test_los_guidance COMMAND test_los_guidance)

add_executable(test_reference_paths
    tests/test_reference_paths.cpp
)
target_link_libraries(test_reference_paths PRIVATE auv_core)
add_test(NAME test_reference_paths COMMAND test_reference_paths)

add_executable(test_path_following
    tests/test_path_following.cpp
)
target_link_libraries(test_path_following PRIVATE auv_core)
add_test(NAME test_path_following COMMAND test_path_following)

add_executable(test_curved_path_following
    tests/test_curved_path_following.cpp
)
target_link_libraries(test_curved_path_following PRIVATE auv_core)
add_test(NAME test_curved_path_following COMMAND test_curved_path_following)
