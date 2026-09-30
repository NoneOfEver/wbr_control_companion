cmake_minimum_required(VERSION 3.16)
project(wbr_control_companion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

option(WBR_ENABLE_IGH "Build the IgH EtherCAT backend when libethercat is available" ON)
option(WBR_ENABLE_SOEM "Build the SOEM backend when libsoem is available" ON)

add_library(wbr_leg_mpc
    src/leg_mpc.cpp
)
target_include_directories(wbr_leg_mpc PUBLIC include)
target_link_libraries(wbr_leg_mpc PUBLIC Eigen3::Eigen)
target_compile_features(wbr_leg_mpc PUBLIC cxx_std_17)
target_compile_options(wbr_leg_mpc PRIVATE -Wall -Wextra -Wpedantic)

add_library(wbr_mpc_link src/mpc_link.cpp)
target_include_directories(wbr_mpc_link PUBLIC include)
target_compile_features(wbr_mpc_link PUBLIC cxx_std_17)
target_compile_options(wbr_mpc_link PRIVATE -Wall -Wextra -Wpedantic)

add_library(wbr_ethercat
    src/ethercat_mpc_safety.cpp
    src/ethercat_protocol.cpp
    src/ethercat_transport_mock.cpp
    src/ethercat_transport_igh.cpp
    src/ethercat_transport_soem.cpp
    src/realtime_loop.cpp
)
target_include_directories(wbr_ethercat PUBLIC include)
target_compile_features(wbr_ethercat PUBLIC cxx_std_17)
target_compile_options(wbr_ethercat PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(wbr_ethercat PUBLIC Threads::Threads)

if(WBR_ENABLE_IGH)
  find_path(IGH_INCLUDE_DIR ecrt.h PATHS /usr/local/include)
  find_library(IGH_LIBRARY ethercat PATHS /usr/local/lib)
  if(IGH_INCLUDE_DIR AND IGH_LIBRARY)
    target_compile_definitions(wbr_ethercat PRIVATE WBR_HAVE_IGH=1)
    target_include_directories(wbr_ethercat PRIVATE ${IGH_INCLUDE_DIR})
    target_link_libraries(wbr_ethercat PRIVATE ${IGH_LIBRARY})
    message(STATUS "IgH EtherCAT backend enabled")
  else()
    message(STATUS "IgH EtherCAT backend disabled: ecrt.h/libethercat not found")
  endif()
endif()

if(WBR_ENABLE_SOEM)
  find_path(SOEM_INCLUDE_DIR soem/ethercat.h)
  find_library(SOEM_LIBRARY NAMES soem SOEM)
  if(SOEM_INCLUDE_DIR AND SOEM_LIBRARY)
    target_compile_definitions(wbr_ethercat PRIVATE WBR_HAVE_SOEM=1)
    # SOEM 1.4 installs ethercat.h under include/soem, while its public
    # headers include each other without the soem/ prefix.
    target_include_directories(wbr_ethercat SYSTEM PRIVATE
      ${SOEM_INCLUDE_DIR}
      ${SOEM_INCLUDE_DIR}/soem
    )
    target_link_libraries(wbr_ethercat PRIVATE ${SOEM_LIBRARY})
    message(STATUS "SOEM backend enabled")
  else()
    message(STATUS "SOEM backend disabled: headers/library not found")
  endif()
endif()

add_executable(ethercat_bench src/ethercat_bench.cpp)
target_link_libraries(ethercat_bench PRIVATE wbr_ethercat)

add_executable(ethercat_mpc src/ethercat_mpc.cpp)
target_link_libraries(ethercat_mpc PRIVATE wbr_ethercat wbr_leg_mpc)

add_executable(leg_mpc_demo src/main.cpp)
target_link_libraries(leg_mpc_demo PRIVATE wbr_leg_mpc)

enable_testing()
add_executable(leg_mpc_test tests/leg_mpc_test.cpp)
target_link_libraries(leg_mpc_test PRIVATE wbr_leg_mpc)
add_test(NAME leg_mpc_test COMMAND leg_mpc_test)

add_executable(mpc_link_test tests/mpc_link_test.cpp)
target_link_libraries(mpc_link_test PRIVATE wbr_mpc_link)
add_test(NAME mpc_link_test COMMAND mpc_link_test)

add_executable(ethercat_protocol_test tests/ethercat_protocol_test.cpp)
target_link_libraries(ethercat_protocol_test PRIVATE wbr_ethercat)
add_test(NAME ethercat_protocol_test COMMAND ethercat_protocol_test)
