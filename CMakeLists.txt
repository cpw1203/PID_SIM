cmake_minimum_required(VERSION 3.2)
set(CMAKE_BUILD_TYPE Debug)
project(PID VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(PID ${CMAKE_SOURCE_DIR}/main.cxx)
include_directories(PID include/)

# Find Qt6 packages
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Charts)

# Link Qt6 libraries
  target_link_libraries(PID 
      Qt5::Charts
      Qt5::Core
      Qt5::Gui
      Qt5::Widgets
      )

