cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(TagSimulator)

add_executable(TagSimulator
    TagSimulator.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(TagSimulator
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(TagSimulator PRIVATE -Wall -Wextra)
else()
    add_compile_options(TagSimulator PRIVATE -WX -W2)
endif()
