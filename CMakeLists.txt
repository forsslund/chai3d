#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2016, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: seb $
#  $Date: 2016-11-01 19:39:15 +0100 (Tue, 01 Nov 2016) $
#  $Rev: 2152 $


#
# configuration
#

cmake_minimum_required (VERSION 3.0.0)
project (CHAI3D)

# version info
file (READ ${PROJECT_SOURCE_DIR}/src/version VERSION)
STRING (REGEX REPLACE ".*MAJOR=([0-9]+).*" "\\1" MAJOR_VERSION "${VERSION}")
STRING (REGEX REPLACE ".*MINOR=([0-9]+).*" "\\1" MINOR_VERSION "${VERSION}")
STRING (REGEX REPLACE ".*RELEASE=([0-9]+).*" "\\1" RELEASE_VERSION "${VERSION}")
set (PROJECT_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${RELEASE_VERSION})

# platform detection
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
  set (OS "win")
  if (${CMAKE_CL_64})
    set (ARCH "x64")
  else ()
    set (ARCH "Win32")
  endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux)
  set (OS "lin")
  if (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set (ARCH "x86_64")
  else ()
    set (ARCH "i686")
  endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set (OS "mac")
  set (ARCH "x86_64")
endif ()

# enforce build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release CACHE STRING "Setting build mode to Release" FORCE)
endif()

# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})

# OpenGL dependency
find_package (OpenGL REQUIRED)
include_directories (OPENGL_INCLUDE_DIR)

# on non-Windows, DHD dependency
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES Windows)
  include_directories (${PROJECT_SOURCE_DIR}/external/DHD/include)
  if(${CMAKE_GENERATOR} STREQUAL Xcode)
    set (DHD_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/external/DHD/lib/${OS})
  else()
    set (DHD_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/external/DHD/lib/${OS}-${ARCH})
  endif()
  link_directories (${DHD_LIBRARY_DIRS})
endif ()

# Windows global build options
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)

  # VisualStudio compiler
  if (MSVC)
    add_definitions (-D_CRT_SECURE_NO_DEPRECATE)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP")
    if (${CMAKE_CL_64})
      add_definitions (-DWIN64)
    else ()
      add_definitions (-DWIN32)
    endif ()

  # MinGW compiler
  elseif (MINGW)
    add_definitions (-DWIN32)
    add_definitions (-DHAVE_GCC_DESTRUCTOR)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wno-deprecated -std=c++0x")
    set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -march=native -Wno-deprecated")
  endif ()

# Linux global build options
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux)
  add_definitions (-DLINUX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -march=native -Wno-deprecated -std=c++0x")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC -march=native -Wno-deprecated")

# Mac OS X global build options
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  add_definitions (-DMACOSX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qunused-arguments -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Qunused-arguments -Wno-deprecated")
endif ()


#
# static library
#

# header search path
include_directories (${PROJECT_SOURCE_DIR}/src)
include_directories (${PROJECT_SOURCE_DIR}/external/Eigen)
include_directories (${PROJECT_SOURCE_DIR}/external/glew/include)
include_directories (${PROJECT_SOURCE_DIR}/external/giflib/include)
include_directories (${PROJECT_SOURCE_DIR}/external/lib3ds/include)
include_directories (${PROJECT_SOURCE_DIR}/external/libjpeg/include)
include_directories (${PROJECT_SOURCE_DIR}/external/libpng/include)
include_directories (${PROJECT_SOURCE_DIR}/external/openal/include)
include_directories (${PROJECT_SOURCE_DIR}/external/openal/OpenAL32/Include)
include_directories (${PROJECT_SOURCE_DIR}/external/openal/Alc)
include_directories (${PROJECT_SOURCE_DIR}/external/pugixml/include)
include_directories (${PROJECT_SOURCE_DIR}/external/theoraplayer/include/theoraplayer)
include_directories (${PROJECT_SOURCE_DIR}/external/theoraplayer/external/ogg/include)
include_directories (${PROJECT_SOURCE_DIR}/external/theoraplayer/external/theora/include)
include_directories (${PROJECT_SOURCE_DIR}/external/theoraplayer/external/vorbis/include)
include_directories (${PROJECT_SOURCE_DIR}/external/theoraplayer/external/vorbis/lib)
include_directories (${PROJECT_SOURCE_DIR}/external/theoraplayer/src/Theora)

# static library source files
file (GLOB_RECURSE source RELATIVE ${CHAI3D_SOURCE_DIR} ${CHAI3D_SOURCE_DIR}/src/*.cpp ${CHAI3D_SOURCE_DIR}/src/*.c ${CHAI3D_SOURCE_DIR}/src/*.h)
#file (GLOB_RECURSE source RELATIVE ${CHAI3D_SOURCE_DIR} ${CHAI3D_SOURCE_DIR}/src/*.cpp ${CHAI3D_SOURCE_DIR}/src/*.h)

# static library external dependencies source files
file (GLOB_RECURSE source_eigen     RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/Eigen/Eigen/*)
file (GLOB_RECURSE source_gif       RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/giflib/*.c       ${PROJECT_SOURCE_DIR}/external/giflib/*.h)
file (GLOB_RECURSE source_glew      RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/glew/*.c         ${PROJECT_SOURCE_DIR}/external/glew/*.h)
file (GLOB_RECURSE source_3ds       RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/lib3ds/*.c       ${PROJECT_SOURCE_DIR}/external/lib3ds/*.h)
file (GLOB_RECURSE source_jpeg      RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/libjpeg/*.c      ${PROJECT_SOURCE_DIR}/external/libjpeg/*.h)
file (GLOB_RECURSE source_png       RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/libpng/*.c       ${PROJECT_SOURCE_DIR}/external/libpng/*.h)
file (GLOB_RECURSE source_pugixml   RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/pugixml/*.c      ${PROJECT_SOURCE_DIR}/external/pugixml/*.h)
file (GLOB_RECURSE source_openal    RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/openal/*.c       ${PROJECT_SOURCE_DIR}/external/openal/*.h)
file (GLOB_RECURSE source_theora    RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/theoraplayer/*.c ${PROJECT_SOURCE_DIR}/external/theoraplayer/*.cpp ${PROJECT_SOURCE_DIR}/external/theoraplayer/*.h)
file (GLOB_RECURSE source_hidlib    RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/hidlib/*.c       ${PROJECT_SOURCE_DIR}/external/hidlib/*.h )

# platform-specific source files adjustments
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
  file (GLOB_RECURSE source_theora_exclude RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/theoraplayer/external/theora/lib/x86/*.c)
else ()
  file (GLOB_RECURSE source_theora_exclude RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/theoraplayer/external/theora/lib/x86_vc/*.c)
endif ()
list (REMOVE_ITEM source_theora ${source_theora_exclude})

# group source files (MSVC likes this)
foreach (FILE ${source} ${source_eigen} ${source_gif} ${source_glew} ${source_3ds} ${source_jpeg} ${source_png} ${source_pugixml} ${source_openal} ${source_theora})
  get_filename_component (PARENT_DIR "${FILE}" PATH)
  string (REGEX REPLACE "(\\./)?(src|include)/?" "" GROUP "${PARENT_DIR}")
  string (REPLACE "/" "\\" GROUP "${GROUP}")
  source_group ("${GROUP}" FILES "${FILE}")
endforeach ()

# build flags
add_definitions (${PROJECT_DEFINITIONS} -DGLEW_STATIC -DAL_ALEXT_PROTOTYPES -DAL_BUILD_LIBRARY -DAL_LIBTYPE_STATIC -D__THEORA -D_LIB)

###############################################################
# JONAS TRYING TO ADD LINKING AGAINST BOOST
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
    SET(BOOST "C:/boost_1_63_0")
    SET(DR "release")
    IF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(DR "debug")
    ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")
    SET(VS_VER "14")
    include_directories (${BOOST})
    SET(A "${BOOST}/bin.v2/libs")
    SET(B "/build/msvc-${VS_VER}.0/${DR}/address-model-64/link-static/threading-multi/")
    link_directories(${A}/system${B})
    link_directories(${A}/date_time${B})
    link_directories(${A}/regex${B})
    link_directories(${A}/context${B})
    link_directories(${A}/coroutine${B})
    link_directories(${A}/thread${B})
    link_directories(${A}/chrono${B})
    add_definitions(-DC_ENABLE_WOODEN_DEVICE_SUPPORT)
    #add_definitions(-DBOOST_ASIO_NO_DEFAULT_LINKED_LIBS)
    set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${A}/system${B})
    set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${A}/date_time${B})
    set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${A}/regex${B})
    set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${A}/context${B})
    set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${A}/coroutine${B})
    set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${A}/thread${B})
    set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${A}/chrono${B})
    set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} wsock32)
    message("BOOST SYSTEM: " ${A}/system${B})
    #set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} boost_system boost_date_time boost_regex boost_context boost_coroutine boost_thread boost_chrono)
    #
    #
    #  DEFAULT CMAKE WAY (DIDNT WORK)
    #
    #set(Boost_USE_STATIC_LIBS ON)
    #set(Boost_USE_MULTITHREADED ON)
    #set(Boost_USE_STATIC_RUNTIME ON)
    #set(Boost_NO_BOOST_CMAKE ON)
    #set(BOOST_ROOT "C:/boost_1_63_0")
    #find_package(Boost 1.63.0 COMPONENTS system date_time regex context coroutine thread chrono)
    #include_directories ("C:/boost_1_63_0")
    #message(STATUS "Boost includes: " ${Boost_FOUND} ${Boost_INCLUDE_DIR})
    #set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS}  Boost_INCLUDE_DIRS)#"c:/boost_1_63_0" )
    #set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} ${Boost_LIBRARIES})# boost_system boost_date_time boost_regex boost_context boost_coroutine boost_thread boost_chrono)
    #add_definitions(-DC_ENABLE_WOODEN_DEVICE_SUPPORT)
    #add_definitions(-DBOOST_ASIO_NO_DEFAULT_LINKED_LIBS)
    #add_definitions(-DWIN32_LEAN_AND_MEAN)
endif()
###############################################################



# static library
add_library (chai3d STATIC ${source} ${source_hidlib} ${source_eigen} ${source_gif} ${source_glew} ${source_3ds} ${source_jpeg} ${source_png} ${source_pugixml} ${source_openal} ${source_theora} )

# library exports
set (CHAI3D_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/external/Eigen" "${PROJECT_SOURCE_DIR}/external/glew/include" "${OPENGL_INCLUDE_DIR}")
set (CHAI3D_LIBRARIES chai3d ${OPENGL_LIBRARIES})
set (CHAI3D_DEFINITIONS ${PROJECT_DEFINITIONS})
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES Windows)
  set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} drd)
  set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${DHD_LIBRARY_DIRS})
else ()
  set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} winmm)
endif ()
if (${OS} MATCHES lin)
  set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} usb-1.0 rt pthread dl)
elseif (${OS} MATCHES mac)
  set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} "-framework CoreFoundation" "-framework IOKit" "-framework CoreServices" "-framework CoreAudio" "-framework AudioToolbox" "-framework AudioUnit")
endif ()
set (CHAI3D_SOURCE_DIR ${PROJECT_SOURCE_DIR})


#
# optional extras
#
#set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} `pkg-config libudev --libs` -lrt)
if (${OS} MATCHES lin)
    set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} "${PROJECT_SOURCE_DIR}/external/s826/lib" "${PROJECT_SOURCE_DIR}/external/libhid")
    link_directories ("${PROJECT_SOURCE_DIR}/external/s826/lib")
    set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} udev 826_64)
    #add_definitions(-DC_ENABLE_WOODEN_DEVICE_SUPPORT)

    # new fsHapticDevice
    set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES}  rt boost_system boost_date_time boost_regex boost_context boost_coroutine boost_thread boost_chrono)

endif ()

# Remote
#set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} "${PROJECT_SOURCE_DIR}/external/libremotehaptics" )
#link_directories ("${PROJECT_SOURCE_DIR}/external/libremotehaptics")
#set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} "c:/Qt58/5.8/msvc2015_64/lib" )
#set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} remotehaptics Qt5Core Qt5Network)


#add_definitions(-DC_ENABLE_REMOTE_DEVICE_SUPPORT)

# GLFW
if (EXISTS ${PROJECT_SOURCE_DIR}/extras/GLFW)
  add_subdirectory (${PROJECT_SOURCE_DIR}/extras/GLFW)
endif ()


#
# executables
#

# examples
if (EXISTS ${PROJECT_SOURCE_DIR}/examples)
  add_subdirectory (${PROJECT_SOURCE_DIR}/examples)
endif ()

# utilities
if (EXISTS ${PROJECT_SOURCE_DIR}/utils)
  add_subdirectory (${PROJECT_SOURCE_DIR}/utils)
endif ()


#
# export package
#

# export package for use from the build tree
export (TARGETS chai3d FILE ${PROJECT_BINARY_DIR}/CHAI3DTargets.cmake)
export (PACKAGE CHAI3D)

# package definitions
set (CONF_INCLUDE_DIRS ${CHAI3D_INCLUDE_DIRS})
set (CONF_LIBRARIES    ${CHAI3D_LIBRARIES})
set (CONF_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS})
set (CONF_DEFINITIONS  ${CHAI3D_DEFINITIONS})
set (CONF_SOURCE_DIR   ${CHAI3D_SOURCE_DIR})

# package configuration and version files
configure_file (CHAI3DConfig.cmake.in "${PROJECT_BINARY_DIR}/CHAI3DConfig.cmake" @ONLY)
configure_file (CHAI3DConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/CHAI3DConfigVersion.cmake" @ONLY)
