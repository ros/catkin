# BUILD_SHARED_LIBS is a global cmake variable (usually defaults to on) 
# that determines the build type of libraries:
#   http://www.cmake.org/cmake/help/cmake-2-8-docs.html#variable:BUILD_SHARED_LIBS
# It defaults to shared.

# Make sure this is already defined as a cached variable (@sa tools/libraries.cmake)
if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
endif()

# Windows/cmake make things difficult if building dll's. 
# By default:
#   .dll -> CMAKE_RUNTIME_OUTPUT_DIRECTORY
#   .exe -> CMAKE_RUNTIME_OUTPUT_DIRECTORY
#   .lib -> CMAKE_LIBRARY_OUTPUT_DIRECTORY
#
# Subsequently, .dll's and .exe's use the same variable and by 
# default must be installed to the same place. Which is not
# what we want for catkin. We wish:
#
#   .dll -> CATKIN_GLOBAL_BIN_DESTINATION
#   .exe -> CATKIN_PACKAGE_BIN_DESTINATION
#   .lib -> CATKIN_PACKAGE_LIB_DESTINATION
#
# Since we can't put CMAKE_RUNTIME_OUTPUT_DIRECTORY to
# two values at once, we have this ugly workaround here.
#
# Note - we want to move away from redefining
# add_library style calls, but necessary until a better solution
# is available for windows. Alternatives are to set_target_properties
# on every lib (painful) or to make exe's public (name conflicts
# bound to arise).
#
# Another feature that would be desirable, is to install .pdb's for
# debugging along with the library. Can't do that here as we do not
# know for sure if the library target is intended for installation
# or not. Might a good idea to have a script that searches for all
# pdb's under CATKIN_DEVEL_PREFIX and copies them over at the end
# of the cmake build.
if(BUILD_SHARED_LIBS)
  if(WIN32)
    function(add_library library)
      # Check if its an external, imported library (e.g. boost libs via cmake module definition)
      list(FIND ARGN "IMPORTED" FIND_IMPORTED)
      list(FIND ARGN "ALIAS" FIND_ALIAS)
      _add_library(${ARGV0} ${ARGN})
      if(${FIND_IMPORTED} EQUAL -1 AND ${FIND_ALIAS} EQUAL -1)
        set_target_properties(${ARGV0} 
          PROPERTIES 
              RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}
              LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
              ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
        )
      endif()
    endfunction()
  endif()
endif()

# It is encouraged to follow this guide to enable exports for dll's in a cross-platform way:
#   http://wiki.ros.org/win_ros/Contributing/Dll%20Exports
# however, since not every project has implemented import/export macros, enable CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS as a workaround
#   https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
if(BUILD_SHARED_LIBS)
  if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  endif()
endif()

# For Windows, add definitions to exclude defining common names macros that cause name collision
if(WIN32)
  # enable Math Constants (https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants)
  add_definitions(-D_USE_MATH_DEFINES)

  # do not define STRICT macros (minwindef.h or boost\winapi\basic_types.hpp)
  add_definitions(-DNO_STRICT)

  # do not define min/max macros
  add_definitions(-DNOMINMAX)

  # do not define STRICT macros (qtgui\qwindowdefs_win.h)
  add_definitions(-DQ_NOWINSTRICT)

  # keep minimum windows headers inclusion
  add_definitions(-DWIN32_LEAN_AND_MEAN)

  # explicitly enable C++ to compile wrapper
  enable_language(CXX)
endif()

if(MSVC)
  # https://blogs.msdn.microsoft.com/vcblog/2018/04/09/msvc-now-correctly-reports-__cplusplus/
  add_compile_options(/Zc:__cplusplus)
endif()

#
# Add Python executable wrapper around Python scripts on Windows.
#
# Python scripts with (or without) .py extension are not executable on Windows
# due to lack of shebang support.
#
# :param SCRIPT_NAME: Python script name that needs a wrapper
# :type SCRIPT_NAME: string
# :param TARGET_NAME: build target name
# :type TARGET_NAME: string
# :param DESTINATION: install destination for the build target
# :type DESTINATION: string
#
# @public
#
function(add_python_executable)
  # Code courtesy of Yujin Robot.
  # Derived from https://github.com/ros-windows/win_ros
  cmake_parse_arguments(
    ARG "" "SCRIPT_NAME;TARGET_NAME;DESTINATION" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "add_python_executable() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(WIN32)
    set(
      WRAPPER_SOURCE
      "${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/add_python_executable/${ARG_TARGET_NAME}/${ARG_SCRIPT_NAME}.cpp")
    configure_file(
      "${catkin_EXTRAS_DIR}/templates/python_win32_wrapper.cpp.in"
      "${WRAPPER_SOURCE}"
      @ONLY)

    add_executable(${ARG_TARGET_NAME} "${WRAPPER_SOURCE}")

    # The actual file name of the executable built on Windows will be ${ARG_SCRIPT_NAME}.exe according to OUTPUT_NAME
    set_target_properties(
      ${ARG_TARGET_NAME} PROPERTIES
      OUTPUT_NAME "${ARG_SCRIPT_NAME}"
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/windows_wrappers/${ARG_TARGET_NAME}")

    install(
      TARGETS ${ARG_TARGET_NAME}
      RUNTIME DESTINATION "${ARG_DESTINATION}")
  endif()
endfunction()
