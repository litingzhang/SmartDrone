cmake_minimum_required(VERSION 3.18)
project(cm5udp)

add_library(cm5udp SHARED
    native_lib.cpp
)

target_compile_features(cm5udp PRIVATE cxx_std_17)
target_include_directories(cm5udp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cm5udp android log)
