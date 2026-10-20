cmake_minimum_required(VERSION 3.16)
project(chassis_controller_native LANGUAGES CXX)
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
 message(FATAL_ERROR "Build the native controller in Orb Ubuntu or on NUC (Linux)")
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
set(generated ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${generated})
foreach(name chassis motors remote_input hi91_imu)
 if(name STREQUAL motors)
  set(namespace platform::motors_params)
 else()
  set(namespace modules::${name}_params)
 endif()
 add_custom_command(OUTPUT ${generated}/${name}_params_generated.h
   COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_params.py
   --input ${CMAKE_CURRENT_SOURCE_DIR}/config/${name}_params.yaml
   --schema ${CMAKE_CURRENT_SOURCE_DIR}/config/${name}_params.schema.yaml
   --namespace ${namespace} --output ${generated}/${name}_params_generated.h
   DEPENDS tools/generate_params.py config/${name}_params.yaml config/${name}_params.schema.yaml)
 list(APPEND params ${generated}/${name}_params_generated.h)
endforeach()
file(GLOB core_sources CONFIGURE_DEPENDS control/*.cpp control/*.cc)
file(GLOB codec_sources CONFIGURE_DEPENDS include/protocols/motors/*.cpp include/protocols/imu/*.cpp include/protocols/remote_input/*.cpp include/protocols/referee/*.cpp)
add_library(chassis_core ${core_sources} ${codec_sources} ${params})
target_include_directories(chassis_core PUBLIC include control ${generated})
target_link_libraries(chassis_core PUBLIC Eigen3::Eigen)
target_compile_options(chassis_core PRIVATE -Wall -Wextra -Wpedantic)

target_compile_definitions(chassis_core PRIVATE EIGEN_NO_MALLOC EIGEN_DONT_VECTORIZE)
add_library(chassis_runtime runtime/sensor_decoder.cpp runtime/usb_link.cpp ../src/realtime_loop.cpp)
target_include_directories(chassis_runtime PUBLIC runtime ../include)
target_link_libraries(chassis_runtime PUBLIC chassis_core Threads::Threads PkgConfig::LIBUSB)
target_compile_options(chassis_runtime PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_executable(chassis_controller runtime/main.cpp)
target_link_libraries(chassis_controller PRIVATE chassis_runtime)
target_compile_options(chassis_controller PRIVATE -Wall -Wextra -Wpedantic -Werror)
enable_testing()
add_test(NAME production_controller_disabled_smoke COMMAND chassis_controller --simulate --no-rt --seconds 0.1)
add_executable(chassis_contracts tests/contracts.cpp)
target_link_libraries(chassis_contracts PRIVATE chassis_runtime)
target_compile_options(chassis_contracts PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME production_control_and_sensor_contracts COMMAND chassis_contracts)
add_library(production_usb_mock SHARED tests/usb_mock.cpp)
target_link_libraries(production_usb_mock PRIVATE chassis_core Threads::Threads PkgConfig::LIBUSB)
add_test(NAME production_usb_lifecycle COMMAND ${Python3_EXECUTABLE}
 ${CMAKE_CURRENT_SOURCE_DIR}/tests/usb_integration.py $<TARGET_FILE:chassis_controller> $<TARGET_FILE:production_usb_mock>)
set_tests_properties(production_usb_lifecycle PROPERTIES TIMEOUT 35)
# Optional migration validation against the original MCU checkout, never needed
# by the production executable or a fresh standalone host build.
set(WBR_REFERENCE_CHASSIS_DIR "" CACHE PATH "Original MCU chassis control source directory")
if(WBR_REFERENCE_CHASSIS_DIR)
 add_executable(chassis_balance_fixture tests/balance_fixture.cpp)
 target_link_libraries(chassis_balance_fixture PRIVATE chassis_core)
 add_executable(chassis_reference_balance tests/balance_fixture.cpp
  ${WBR_REFERENCE_CHASSIS_DIR}/balance_controller.cpp
  ${WBR_REFERENCE_CHASSIS_DIR}/body_motion_estimator.cpp
  ${WBR_REFERENCE_CHASSIS_DIR}/leg_vmc.cpp
  ${WBR_REFERENCE_CHASSIS_DIR}/leg_kinematics.cc
  ${WBR_REFERENCE_CHASSIS_DIR}/unified_lqr_schedule.cc)
 target_include_directories(chassis_reference_balance PRIVATE
  ${WBR_REFERENCE_CHASSIS_DIR} include ${generated})
 target_link_libraries(chassis_reference_balance PRIVATE Eigen3::Eigen)
 target_compile_definitions(chassis_reference_balance PRIVATE EIGEN_NO_MALLOC EIGEN_DONT_VECTORIZE)
 add_dependencies(chassis_reference_balance chassis_core)
 add_test(NAME production_balance_matches_original COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/compare_reference.py
  $<TARGET_FILE:chassis_reference_balance> $<TARGET_FILE:chassis_balance_fixture>)
endif()

# Independent capture tool; never linked into the production controller.
add_executable(uart_forwarding measurements/uart_forwarding.cpp)
target_link_libraries(uart_forwarding PRIVATE chassis_runtime)
target_compile_options(uart_forwarding PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME uart_forwarding_capture COMMAND ${Python3_EXECUTABLE}
 ${CMAKE_CURRENT_SOURCE_DIR}/tests/uart_capture.py
 $<TARGET_FILE:uart_forwarding> $<TARGET_FILE:production_usb_mock>)
set_tests_properties(uart_forwarding_capture PROPERTIES TIMEOUT 15)
add_test(NAME uart_capture_offline_analysis COMMAND ${Python3_EXECUTABLE}
 ${CMAKE_CURRENT_SOURCE_DIR}/tests/uart_analysis.py)
add_executable(usb_reconnect_contract tests/usb_reconnect.cpp)
target_link_libraries(usb_reconnect_contract PRIVATE chassis_runtime)
target_compile_options(usb_reconnect_contract PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME production_usb_reconnect COMMAND ${CMAKE_COMMAND} -E env
 LD_PRELOAD=$<TARGET_FILE:production_usb_mock> WBR_MOCK_STALE_SESSION=1 WBR_MOCK_REQUIRE_PREPOST=1
 $<TARGET_FILE:usb_reconnect_contract>)
set_tests_properties(production_usb_reconnect PROPERTIES TIMEOUT 10)

add_executable(full_load_zero measurements/full_load/main.cpp measurements/full_load/capture.cpp)
target_link_libraries(full_load_zero PRIVATE chassis_runtime)
target_compile_options(full_load_zero PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME full_load_zero_capture COMMAND ${Python3_EXECUTABLE}
 ${CMAKE_CURRENT_SOURCE_DIR}/tests/full_load_capture.py
 $<TARGET_FILE:full_load_zero> $<TARGET_FILE:production_usb_mock>)
set_tests_properties(full_load_zero_capture PROPERTIES TIMEOUT 30)
