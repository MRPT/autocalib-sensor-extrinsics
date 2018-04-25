# Include directories

set(SOURCES 
   main.cpp
   mainwindow.cpp
   mainwindow.h)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

add_executable(autocalib_sensor_extrinsics ${SOURCES})

target_link_libraries(autocalib_sensor_extrinsics Qt5::Widgets)
