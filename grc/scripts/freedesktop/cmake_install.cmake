# Install script for directory: /home/lx/gnuradio/grc/scripts/freedesktop

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "grc")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gnuradio/grc/freedesktop" TYPE FILE FILES
    "/home/lx/gnuradio/grc/scripts/freedesktop/grc-icon-256.png"
    "/home/lx/gnuradio/grc/scripts/freedesktop/grc-icon-128.png"
    "/home/lx/gnuradio/grc/scripts/freedesktop/grc-icon-64.png"
    "/home/lx/gnuradio/grc/scripts/freedesktop/grc-icon-48.png"
    "/home/lx/gnuradio/grc/scripts/freedesktop/grc-icon-32.png"
    "/home/lx/gnuradio/grc/scripts/freedesktop/grc-icon-24.png"
    "/home/lx/gnuradio/grc/scripts/freedesktop/grc-icon-16.png"
    "/home/lx/gnuradio/grc/scripts/freedesktop/gnuradio-grc.xml"
    "/home/lx/gnuradio/grc/scripts/freedesktop/gnuradio-grc.desktop"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "grc")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/libexec/gnuradio" TYPE PROGRAM FILES "/home/lx/gnuradio/gr-lora/grc/scripts/freedesktop/grc_setup_freedesktop")
endif()

