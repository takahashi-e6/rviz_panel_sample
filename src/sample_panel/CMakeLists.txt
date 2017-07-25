## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(sample_panel)

find_package(catkin REQUIRED COMPONENTS rviz message_generation)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# headerを同ディレクトリに入れないと、rviz上にロード出来なくなる
#include_directories(include)

# for use cpp11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES src/sample.cpp)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SRC_FILES}``.
add_library(${PROJECT_NAME} ${SRC_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} )
## END_TUTORIAL

