cmake_minimum_required(VERSION 2.8.11)

project(autocalib-sensor-extrinsics)

find_package(Qt5Widgets)

set(SOURCES 
   autocalib-gui/main.cpp
   autocalib-gui/mainwindow.cpp
   autocalib-gui/mainwindow.h)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

add_executable(autocalib_sensor_extrinsics ${SOURCES})

target_link_libraries(autocalib_sensor_extrinsics Qt5::Widgets)
