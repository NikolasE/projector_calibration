##############################################################################
# Rosbuild2
##############################################################################

if(ROSBUILD)
  include(rosbuild.cmake OPTIONAL)
  return()
endif()

##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

##############################################################################
# Ros Initialisation
##############################################################################



include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

##############################################################################
# Qt Environment
##############################################################################

rosbuild_include(qt_build qt-ros)
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-unknown-pragmas -Wno-unused-function -fpch-preprocess")

ADD_DEFINITIONS(-DQT_NO_KEYWORDS )

#############################
# Qt ########################
#############################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

include_directories(${QT_QTOPENGL_INCLUDE_DIR})

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

 
##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/projector_calibration/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
#QT4_WRAP_CPP(include/projector_calibration/gl_viewer.h)

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

   
option(WITH_LIBGEOMETRY "Inclusion of LibGeometry")
SET(LibGeometry_Base "/work/home/engelhan/code/libgeometry" CACHE FILEPATH "Location of LibGeometry")

if (WITH_LIBGEOMETRY)
add_definitions(-DWITH_LIBGEOMETRY)
include_directories(${LibGeometry_Base})
include_directories(${LibGeometry_Base}/WildMagic4/SDK/Include)
include_directories( ${includes}) #remove?
include_directories(${LibGeometry_Base}/Testing ${LibGeometry_Base}/mesh ${LibGeometry_Base}/base)
include_directories(${LibGeometry_Base}/parameterization ${LibGeometry_Base}/geometry ${LibGeometry_Base}/pointset)
include_directories(${LibGeometry_Base}/spatial)
include_directories( ${includes})
endif (WITH_LIBGEOMETRY)

#${OPENGL_LIBRARIES} ${GLUT_LIBRARIES}
   
  

   
rosbuild_add_executable(calibration_app ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(calibration_app ${QT_LIBRARIES} rgbd_utils gl_viewer GLEW GL GLU   ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})

if (WITH_LIBGEOMETRY)
target_link_libraries(calibration_app  ${GEO_FOLDER}/Testing/liblibGeometry.a ${GEO_FOLDER}/WildMagic4/SDK/Library/Release/libWm4Foundation.a )
endif (WITH_LIBGEOMETRY)

#rosbuild_add_library(projector_calibration  src/projector_calibrator.cpp)
