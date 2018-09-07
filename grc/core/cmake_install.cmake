# Install script for directory: /home/lx/gnuradio/grc/core

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/gnuradio/grc/core" TYPE FILE FILES
    "/home/lx/gnuradio/grc/core/ParseXML.py"
    "/home/lx/gnuradio/grc/core/Element.py"
    "/home/lx/gnuradio/grc/core/Param.py"
    "/home/lx/gnuradio/grc/core/Platform.py"
    "/home/lx/gnuradio/grc/core/Constants.py"
    "/home/lx/gnuradio/grc/core/Messages.py"
    "/home/lx/gnuradio/grc/core/Config.py"
    "/home/lx/gnuradio/grc/core/Block.py"
    "/home/lx/gnuradio/grc/core/FlowGraph.py"
    "/home/lx/gnuradio/grc/core/__init__.py"
    "/home/lx/gnuradio/grc/core/Port.py"
    "/home/lx/gnuradio/grc/core/Connection.py"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "grc")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/gnuradio/grc/core" TYPE FILE FILES
    "/home/lx/gnuradio/gr-lora/grc/core/ParseXML.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Element.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Param.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Platform.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Constants.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Messages.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Config.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Block.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/FlowGraph.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/__init__.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Port.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/Connection.pyc"
    "/home/lx/gnuradio/gr-lora/grc/core/ParseXML.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Element.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Param.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Platform.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Constants.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Messages.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Config.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Block.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/FlowGraph.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/__init__.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Port.pyo"
    "/home/lx/gnuradio/gr-lora/grc/core/Connection.pyo"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "grc")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/gnuradio/grc/core" TYPE FILE FILES
    "/home/lx/gnuradio/grc/core/flow_graph.dtd"
    "/home/lx/gnuradio/grc/core/block_tree.dtd"
    "/home/lx/gnuradio/grc/core/block.dtd"
    "/home/lx/gnuradio/grc/core/domain.dtd"
    "/home/lx/gnuradio/grc/core/default_flow_graph.grc"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lx/gnuradio/gr-lora/grc/core/generator/cmake_install.cmake")
  include("/home/lx/gnuradio/gr-lora/grc/core/utils/cmake_install.cmake")

endif()

