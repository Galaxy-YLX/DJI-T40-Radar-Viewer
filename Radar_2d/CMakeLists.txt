cmake_minimum_required(VERSION 3.16)
project(radar2d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Network)

add_executable(radar2d
    src/main.cpp
    src/config.h
    src/sidestream.h
    src/sidestream.cpp
    src/f0decoder.h
    src/f0decoder.cpp
    src/ppiwidget.h
    src/ppiwidget.cpp
    src/mainwindow.h
    src/mainwindow.cpp
)

target_link_libraries(radar2d PRIVATE Qt6::Widgets Qt6::Network)
