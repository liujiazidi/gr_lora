# Install script for directory: /home/lx/gnuradio/grc/blocks

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gnuradio/grc/blocks" TYPE FILE FILES
    "/home/lx/gnuradio/grc/blocks/bus_source.xml"
    "/home/lx/gnuradio/grc/blocks/virtual_sink.xml"
    "/home/lx/gnuradio/grc/blocks/virtual_source.xml"
    "/home/lx/gnuradio/grc/blocks/import.xml"
    "/home/lx/gnuradio/grc/blocks/options.xml"
    "/home/lx/gnuradio/grc/blocks/bus_sink.xml"
    "/home/lx/gnuradio/grc/blocks/bus_structure_source.xml"
    "/home/lx/gnuradio/grc/blocks/variable_config.xml"
    "/home/lx/gnuradio/grc/blocks/note.xml"
    "/home/lx/gnuradio/grc/blocks/pad_source.xml"
    "/home/lx/gnuradio/grc/blocks/pad_sink.xml"
    "/home/lx/gnuradio/grc/blocks/variable_function_probe.xml"
    "/home/lx/gnuradio/grc/blocks/bus_structure_sink.xml"
    "/home/lx/gnuradio/grc/blocks/gr_stream_domain.xml"
    "/home/lx/gnuradio/grc/blocks/epy_module.xml"
    "/home/lx/gnuradio/grc/blocks/dummy.xml"
    "/home/lx/gnuradio/grc/blocks/gr_message_domain.xml"
    "/home/lx/gnuradio/grc/blocks/variable.xml"
    "/home/lx/gnuradio/grc/blocks/block_tree.xml"
    "/home/lx/gnuradio/grc/blocks/epy_block.xml"
    "/home/lx/gnuradio/grc/blocks/parameter.xml"
    "/home/lx/gnuradio/gr-lora/grc/blocks/variable_struct.xml"
    )
endif()

