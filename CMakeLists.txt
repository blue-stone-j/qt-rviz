cmake_minimum_required(VERSION 3.10)
project(qt_rviz)

# Redefine install() to do nothing
# macro(install)
# # Do nothing
# endmacro()
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

include(FindPkgConfig)
find_package(PkgConfig)

find_package(roscpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rviz REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

include(cmake/YAML.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
  include
  ${rviz_INCLUDE_DIRS}
)

qt5_wrap_cpp(MOC_FILES include/display/mainwindow.h)

# qt_rviz
add_executable(qt_rviz_node src/qt_rviz_node.cpp
  src/display/mainwindow.cpp

  # src/files_loader.cpp
  src/publishers.cpp
  ${MOC_FILES}
)

target_compile_options(qt_rviz_node PRIVATE -Wno-register)

target_link_libraries(qt_rviz_node
  ${roscpp_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${rviz_LIBRARIES}

  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  glog gflags

  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL)

# ---------------------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------------------
# this function is imported from cmake 3.18. It may not be avaliable.
# Optionally, restore install() if needed
# macro(install)
# cmake_language(CALL_CMAKE INSTALL ${ARGN})
# endmacro()
# CALL_CMAKEINSTALL ${ARGN}
# unset(install)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)

message(NOTICE ${CMAKE_INSTALL_PREFIX})
install(TARGETS qt_rviz_node
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
install(DIRECTORY
  config
  assets
  DESTINATION ${CMAKE_INSTALL_PREFIX}/
)
