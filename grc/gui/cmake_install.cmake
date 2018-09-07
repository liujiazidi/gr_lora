# Install script for directory: /home/lx/gnuradio/grc/gui

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/gnuradio/grc/gui" TYPE FILE FILES
    "/home/lx/gnuradio/grc/gui/BlockTreeWindow.py"
    "/home/lx/gnuradio/grc/gui/Preferences.py"
    "/home/lx/gnuradio/grc/gui/ActionHandler.py"
    "/home/lx/gnuradio/grc/gui/Dialogs.py"
    "/home/lx/gnuradio/grc/gui/Utils.py"
    "/home/lx/gnuradio/grc/gui/Colors.py"
    "/home/lx/gnuradio/grc/gui/ParserErrorsDialog.py"
    "/home/lx/gnuradio/grc/gui/external_editor.py"
    "/home/lx/gnuradio/grc/gui/FileDialogs.py"
    "/home/lx/gnuradio/grc/gui/Element.py"
    "/home/lx/gnuradio/grc/gui/MainWindow.py"
    "/home/lx/gnuradio/grc/gui/DrawingArea.py"
    "/home/lx/gnuradio/grc/gui/Param.py"
    "/home/lx/gnuradio/grc/gui/Platform.py"
    "/home/lx/gnuradio/grc/gui/Constants.py"
    "/home/lx/gnuradio/grc/gui/Bars.py"
    "/home/lx/gnuradio/grc/gui/StateCache.py"
    "/home/lx/gnuradio/grc/gui/Config.py"
    "/home/lx/gnuradio/grc/gui/Actions.py"
    "/home/lx/gnuradio/grc/gui/VariableEditor.py"
    "/home/lx/gnuradio/grc/gui/NotebookPage.py"
    "/home/lx/gnuradio/grc/gui/Block.py"
    "/home/lx/gnuradio/grc/gui/PropsDialog.py"
    "/home/lx/gnuradio/grc/gui/FlowGraph.py"
    "/home/lx/gnuradio/grc/gui/Executor.py"
    "/home/lx/gnuradio/grc/gui/__init__.py"
    "/home/lx/gnuradio/grc/gui/Port.py"
    "/home/lx/gnuradio/grc/gui/Connection.py"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "grc")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/gnuradio/grc/gui" TYPE FILE FILES
    "/home/lx/gnuradio/gr-lora/grc/gui/BlockTreeWindow.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Preferences.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/ActionHandler.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Dialogs.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Utils.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Colors.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/ParserErrorsDialog.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/external_editor.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/FileDialogs.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Element.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/MainWindow.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/DrawingArea.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Param.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Platform.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Constants.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Bars.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/StateCache.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Config.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Actions.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/VariableEditor.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/NotebookPage.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Block.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/PropsDialog.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/FlowGraph.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Executor.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/__init__.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Port.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/Connection.pyc"
    "/home/lx/gnuradio/gr-lora/grc/gui/BlockTreeWindow.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Preferences.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/ActionHandler.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Dialogs.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Utils.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Colors.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/ParserErrorsDialog.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/external_editor.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/FileDialogs.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Element.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/MainWindow.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/DrawingArea.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Param.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Platform.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Constants.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Bars.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/StateCache.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Config.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Actions.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/VariableEditor.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/NotebookPage.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Block.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/PropsDialog.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/FlowGraph.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Executor.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/__init__.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Port.pyo"
    "/home/lx/gnuradio/gr-lora/grc/gui/Connection.pyo"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "grc")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/gnuradio/grc/gui" TYPE FILE FILES "/home/lx/gnuradio/grc/gui/icon.png")
endif()

