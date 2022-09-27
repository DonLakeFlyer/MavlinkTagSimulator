cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

project(TagSimulator)

add_executable(TagSimulator
    main.cpp
    CommandHandler.cpp
    PulseSimulator.cpp
    UDPPulseReceiver.cpp
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
