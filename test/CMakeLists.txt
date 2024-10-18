cmake_minimum_required(VERSION 3.20)

project(nanopb_test)
set(CMAKE_CXX_STANDARD 20)
add_subdirectory(../components/etl/etl ${CMAKE_BINARY_DIR}/etl)
add_subdirectory(../components/nanopb/nanopb ${CMAKE_BINARY_DIR}/nanopb)
add_subdirectory(simple_log)

set(WHITE_LIST_SRC ../main/src/whitelist.cpp ../main/inc/whitelist.h)

set(BLE_PB_SRC ../components/nanopb/protobuf/ble.pb.c ../components/nanopb/protobuf/ble.pb.h)
add_executable(test main.cpp ${WHITE_LIST_SRC} ${BLE_PB_SRC})
target_include_directories(test PUBLIC ../main/inc)
target_include_directories(test PUBLIC ../components/nanopb/protobuf)
target_link_libraries(test etl::etl protobuf-nanopb-static simple_log)
target_compile_definitions(test PUBLIC SIMPLE_LOG)
