cmake_minimum_required(VERSION 3.16)
project(can_usb_host_realtime LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "This native benchmark targets the Linux NUC")
endif()
set(WBR_COMPANION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../.." CACHE PATH
    "Existing wbr_control_companion source tree")
if(NOT EXISTS "${WBR_COMPANION_DIR}/src/realtime_loop.cpp")
  message(FATAL_ERROR "Set WBR_COMPANION_DIR to the existing companion checkout")
endif()
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
add_executable(can_usb_host_rt main.cpp usb_benchmark.cpp event_waiter.cpp "${WBR_COMPANION_DIR}/src/realtime_loop.cpp")
target_include_directories(can_usb_host_rt PRIVATE "${WBR_COMPANION_DIR}/include")
target_link_libraries(can_usb_host_rt PRIVATE Threads::Threads PkgConfig::LIBUSB)
target_compile_options(can_usb_host_rt PRIVATE -Wall -Wextra -Wpedantic -Werror)
enable_testing()
add_executable(bridge_profile_test bridge_profile_test.cpp)
target_compile_options(bridge_profile_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME bridge_order_and_profile_contract COMMAND bridge_profile_test)
add_executable(echo_packet_test echo_packet_test.cpp)
add_test(NAME echo_packet_wire_contract COMMAND echo_packet_test)

add_executable(event_waiter_test event_waiter_test.cpp event_waiter.cpp)
target_compile_options(event_waiter_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME high_resolution_event_wait COMMAND event_waiter_test)

add_executable(bridge_packet_test bridge_packet_test.cpp)
target_compile_options(bridge_packet_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME bridge_feedback_contract COMMAND bridge_packet_test)

add_executable(bridge_timing_test bridge_timing_test.cpp)
target_compile_options(bridge_timing_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME bridge_timing_contract COMMAND bridge_timing_test)

add_executable(bridge_stream_test bridge_stream_test.cpp)
target_compile_options(bridge_stream_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME bridge_stream_contract COMMAND bridge_stream_test)

add_executable(control_latency_test control_latency_test.cpp)
target_compile_options(control_latency_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME control_latency_identity_contract COMMAND control_latency_test)

add_executable(control_latency_report_test control_latency_report_test.cpp
  "${WBR_COMPANION_DIR}/src/realtime_loop.cpp")
target_include_directories(control_latency_report_test PRIVATE "${WBR_COMPANION_DIR}/include")
target_link_libraries(control_latency_report_test PRIVATE Threads::Threads)
target_compile_options(control_latency_report_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME control_latency_measurement_contract COMMAND control_latency_report_test)

add_executable(feedback_gate_test feedback_gate_test.cpp)
target_compile_options(feedback_gate_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME bounded_feedback_sampling_contract COMMAND feedback_gate_test)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME control_timeline_wrap_and_order
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/control_timeline_test.py)

add_executable(can_usb_rmcs_host_rt rmcs_style_main.cpp rmcs_style_benchmark.cpp
  event_waiter.cpp "${WBR_COMPANION_DIR}/src/realtime_loop.cpp")
target_include_directories(can_usb_rmcs_host_rt PRIVATE "${WBR_COMPANION_DIR}/include")
target_link_libraries(can_usb_rmcs_host_rt PRIVATE Threads::Threads PkgConfig::LIBUSB)
target_compile_options(can_usb_rmcs_host_rt PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(merged_control_test merged_control_test.cpp)
target_compile_options(merged_control_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME merged_control_zero_contract COMMAND merged_control_test)
add_executable(rmcs_style_state_test rmcs_style_state_test.cpp)
target_link_libraries(rmcs_style_state_test PRIVATE Threads::Threads)
target_compile_options(rmcs_style_state_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME rmcs_style_concurrent_state_contract COMMAND rmcs_style_state_test)

# Test-only LD_PRELOAD backend; it never connects to a physical USB device.
add_library(rmcs_usb_mock SHARED rmcs_usb_mock.cpp)
target_link_libraries(rmcs_usb_mock PRIVATE Threads::Threads PkgConfig::LIBUSB)
target_compile_options(rmcs_usb_mock PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME rmcs_style_usb_lifecycle_integration
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/rmcs_style_integration_test.py
    $<TARGET_FILE:can_usb_rmcs_host_rt> $<TARGET_FILE:rmcs_usb_mock>)
set_tests_properties(rmcs_style_usb_lifecycle_integration PROPERTIES TIMEOUT 25)
