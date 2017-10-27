_generate_function_if_testing_is_disabled(
  "catkin_add_gtest"
  "catkin_add_gmock")

#
# Add a GTest based test target.
#
# An executable target is created with the source files, it is linked
# against GTest and added to the set of unit tests.
#
# .. note:: The test can be executed by calling the binary directly
#   or using: ``make run_tests_${PROJECT_NAME}_gtest_${target}``
#
# :param target: the target name
# :type target: string
# :param source_files: a list of source files used to build the test
#   executable
# :type source_files: list of strings
# :param TIMEOUT: currently not supported
# :type TIMEOUT: integer
# :param WORKING_DIRECTORY: the working directory when executing the
#   executable
# :type WORKING_DIRECTORY: string
#
# @public
#
function(catkin_add_gtest target)
  _catkin_add_google_test("gtest" ${target} ${ARGN})
endfunction()

#
# Add a GMock based test target.
#
# An executable target is created with the source files, it is linked
# against GTest and GMock and added to the set of unit tests.
#
# .. note:: The test can be executed by calling the binary directly
#   or using: ``make run_tests_${PROJECT_NAME}_gtest_${target}``
#
# :param target: the target name
# :type target: string
# :param source_files: a list of source files used to build the test
#   executable
# :type source_files: list of strings
# :param TIMEOUT: currently not supported
# :type TIMEOUT: integer
# :param WORKING_DIRECTORY: the working directory when executing the
#   executable
# :type WORKING_DIRECTORY: string
#
# @public
#
function(catkin_add_gmock target)
  _catkin_add_google_test("gmock" ${target} ${ARGN})
endfunction()

#
# This is an internal function, use catkin_add_gtest or catkin_add_gmock
# instead.
#
# :param type: "gtest" or "gmock"
# The remaining arguments are the same as for catkin_add_gtest and
# catkin_add_gmock.
#
function(_catkin_add_google_test type target)
  if (NOT "${type}" STREQUAL "gtest" AND NOT "${type}" STREQUAL "gmock")
    message(FATAL_ERROR
      "Invalid use of _catkin_add_google_test function, "
      "first argument must be 'gtest' or 'gmock'")
  endif()
  _warn_if_skip_testing("catkin_add_${type}")

  # XXX look for optional TIMEOUT argument, #2645
  cmake_parse_arguments(ARG "" "TIMEOUT;WORKING_DIRECTORY" "" ${ARGN})
  if(ARG_TIMEOUT)
    message(WARNING "TIMEOUT argument to catkin_add_${type}() is ignored")
  endif()

  _catkin_add_executable_with_google_test(${type} ${target} ${ARG_UNPARSED_ARGUMENTS} EXCLUDE_FROM_ALL)

  if(TARGET ${target})
    # make sure the target is built before running tests
    add_dependencies(tests ${target})

    # XXX we DONT use rosunit to call the executable to get process control, #1629, #3112
    get_target_property(_target_path ${target} RUNTIME_OUTPUT_DIRECTORY)
    set(cmd "${_target_path}/${target} --gtest_output=xml:${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/gtest-${target}.xml")
    catkin_run_tests_target("gtest" ${target} "gtest-${target}.xml" COMMAND ${cmd} DEPENDENCIES ${target} WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY})
  endif()
endfunction()

#
# Add a GTest executable target.
#
# An executable target is created with the source files, it is linked
# against GTest.
# If you also want to register the executable as a test use
# ``catkin_add_gtest()`` instead.
#
# :param target: the target name
# :type target: string
# :param source_files: a list of source files used to build the test
#   executable
# :type source_files: list of strings
#
# Additionally, the option EXCLUDE_FROM_ALL can be specified.
# @public
#
function(catkin_add_executable_with_gtest target)
  _catkin_add_executable_with_google_test("gtest" ${target} ${ARGN})
endfunction()

#
# Add a GMock executable target.
#
# An executable target is created with the source files, it is linked
# against GTest and GMock.
# If you also want to register the executable as a test use
# ``catkin_add_gtest()`` instead.
#
# :param target: the target name
# :type target: string
# :param source_files: a list of source files used to build the test
#   executable
# :type source_files: list of strings
#
# Additionally, the option EXCLUDE_FROM_ALL can be specified.
# @public
#
function(catkin_add_executable_with_gmock target)
  _catkin_add_executable_with_google_test("gmock" ${target} ${ARGN})
endfunction()

#
# This is an internal function, use catkin_add_executable_with_gtest
# or catkin_add_executable_with_gmock instead.
#
# :param type: "gtest" or "gmock"
# The remaining arguments are the same as for
# catkin_add_executable_with_gtest and
# catkin_add_executable_with_gmock.
#
function(_catkin_add_executable_with_google_test type target)
  if (NOT "${type}" STREQUAL "gtest" AND NOT "${type}" STREQUAL "gmock")
    message(FATAL_ERROR "Invalid use of _catkin_add_executable_google_test function, first argument must be 'gtest' or 'gmock'")
  endif()
  string(TOUPPER "${type}" type_upper)
  if(NOT ${type_upper}_FOUND AND NOT ${type_upper}_FROM_SOURCE_FOUND)
    message(WARNING "skipping ${type} '${target}' in project '${PROJECT_NAME}' because ${type} was not found")
    return()
  endif()

  if(NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    message(FATAL_ERROR "catkin_add_executable_with_${type}() must be called after catkin_package() so that default output directories for the executables are defined")
  endif()

  cmake_parse_arguments(ARG "EXCLUDE_FROM_ALL" "" "" ${ARGN})

  if ("${type}" STREQUAL "gmock")
    # gmock requires gtest headers and libraries
    list(APPEND GMOCK_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})
    list(APPEND GMOCK_LIBRARY_DIRS ${GTEST_LIBRARY_DIRS})
    list(APPEND GMOCK_LIBRARIES ${GTEST_LIBRARIES})
  endif()

  # create the executable, with basic + gtest/gmock build flags
  include_directories(${${type_upper}_INCLUDE_DIRS})
  link_directories(${${type_upper}_LIBRARY_DIRS})
  add_executable(${target} ${ARG_UNPARSED_ARGUMENTS})
  if(ARG_EXCLUDE_FROM_ALL)
    set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()

  assert(${type_upper}_LIBRARIES)
  target_link_libraries(${target} ${${type_upper}_LIBRARIES} ${THREADS_LIBRARY})

  # make sure gtest/gmock is built before the target
  add_dependencies(${target} ${${type_upper}_LIBRARIES})
endfunction()

#
# Find GTest source-only install.
#
# Google recommends distributing GTest as source only, to be built with the same
# flags as that which is being tested.
#
# :param[in] include_paths: Paths to search for GTest includes
# :param[in] src_paths: Paths to search for GTest sources
# :param[out] found: Whether or not GTest was found in the paths provided
# :param[out] base_dir: The base directory containing GTest's CMakeLists.txt
# :param[out] include_dir: The include path to access GTest's headers
# :param[out] lib_dir: The library path to access GTest's libraries
# :param[out] libs: GTest's libraries
# :param[out] main_libs: GTest's main libraries
#
function(catkin_find_gtest_source include_paths
  src_paths found base_dir include_dir lib_dir libs main_libs
)
  # Find the gtest headers
  find_file(_GTEST_INCLUDES "gtest.h"
            PATHS ${include_paths}
            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
  )

  # Find the gtest sources
  find_file(_GTEST_SOURCES "gtest.cc"
            PATHS ${src_paths}
            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
  )

  # If we found gtest, set the variables accordingly
  if(_GTEST_INCLUDES AND _GTEST_SOURCES)
    get_filename_component(SOURCE_DIR ${_GTEST_SOURCES} PATH)
    get_filename_component(BASE_DIR ${SOURCE_DIR} PATH)

    get_filename_component(INCLUDE_DIR ${_GTEST_INCLUDES} PATH)
    get_filename_component(INCLUDE_DIR ${INCLUDE_DIR} PATH)

    set(${found} TRUE PARENT_SCOPE)
    set(${base_dir} ${BASE_DIR} PARENT_SCOPE)
    set(${include_dir} ${INCLUDE_DIR} PARENT_SCOPE)
    set(${lib_dir} ${CMAKE_BINARY_DIR}/gtest PARENT_SCOPE)
    set(${libs} "gtest" PARENT_SCOPE)
    set(${main_libs} "gtest_main" PARENT_SCOPE)
  endif()
endfunction()

#
# Find GMock source-only install.
#
# Google recommends distributing GMock as source only, to be built with the same
# flags as that which is being tested.
#
# :param[in] include_paths: Paths to search for GMock includes
# :param[in] src_paths: Paths to search for GMock sources
# :param[out] found: Whether or not GMock was found in the paths provided
# :param[out] base_dir: The base directory containing GMock's CMakeLists.txt
# :param[out] include_dir: The include path to access GMock's headers
# :param[out] lib_dir: The library path to access GMock's libraries
# :param[out] libs: GMock's libraries
# :param[out] main_libs: GMock's main libraries
#
function(catkin_find_gmock_source
  include_paths src_paths found base_dir include_dir lib_dir libs main_libs
  gtest_found gtest_base_dir gtest_include_dir gtest_lib_dir gtest_libs gtest_main_libs
)
  # Find the gmock headers
  find_file(_GMOCK_INCLUDES "gmock.h"
            PATHS ${include_paths}
            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
  )

  # Find the gmock sources
  find_file(_GMOCK_SOURCES "gmock.cc"
            PATHS ${src_paths}
            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
  )

  # If we found gmock, set the variables accordingly
  if(_GMOCK_INCLUDES AND _GMOCK_SOURCES)
    get_filename_component(SOURCE_DIR ${_GMOCK_SOURCES} PATH)
    get_filename_component(BASE_DIR ${SOURCE_DIR} PATH)

    set(_gtest_include_paths "/usr/include/gtest")
    if(CATKIN_TOPLEVEL)
      # Ensure current workspace is searched before system path
      list(INSERT _gtest_include_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googletest/include/gtest")
    endif()

    set(_gtest_source_paths "/usr/src/gtest/src")
    if(CATKIN_TOPLEVEL)
      # Ensure current workspace is searched before system path
      list(INSERT _gtest_source_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googletest/src")
    endif()

    catkin_find_gtest_source("${_gtest_include_paths}"
                             "${_gtest_source_paths}" _gtest_found
                             _gtest_base_dir _gtest_include_dir _gtest_lib_dir
                             _gtest_libs _gtest_main_libs)

    # If we found gtest, finding gmock succeeded
    if(gtest_found)
      get_filename_component(INCLUDE_DIR ${_GMOCK_INCLUDES} PATH)
      get_filename_component(INCLUDE_DIR ${INCLUDE_DIR} PATH)

      set(${found} TRUE PARENT_SCOPE)
      set(${base_dir} ${BASE_DIR} PARENT_SCOPE)
      set(${include_dir} ${INCLUDE_DIR} PARENT_SCOPE)
      set(${lib_dir} ${CMAKE_BINARY_DIR}/gmock PARENT_SCOPE)
      set(${libs} "gmock" PARENT_SCOPE)
      set(${main_libs} "gmock_main" PARENT_SCOPE)

      set(${gtest_found} ${_gtest_found} PARENT_SCOPE)
      set(${gtest_base_dir} ${_gtest_base_dir} PARENT_SCOPE)
      set(${gtest_include_dir} ${_gtest_include_dir} PARENT_SCOPE)
      set(${gtest_lib_dir} ${_gtest_lib_dir} PARENT_SCOPE)
      set(${gtest_libs} ${_gtest_libs} PARENT_SCOPE)
      set(${gtest_main_libs} ${_gtest_main_libs} PARENT_SCOPE)
    else()
      message(WARNING "Found gmock, but it did not contain gtest! Please ensure gmock is installed correctly.")
    endif()
  endif()
endfunction()

find_package(GMock QUIET)
if(NOT GMOCK_FOUND)
  find_package(GTest QUIET)
  if(NOT GTEST_FOUND)
    # only add gmock/gtest directory once per workspace
    if(NOT TARGET gtest AND NOT TARGET gmock)
      # Fall back to system-installed gmock source (e.g. Ubuntu)
      set(_include_paths "/usr/include/gmock")
      if(CATKIN_TOPLEVEL)
        # Ensure current workspace is searched before system path
        list(INSERT _include_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googlemock/include/gmock")
      endif()

      set(_source_paths "/usr/src/gmock/src")
      if(CATKIN_TOPLEVEL)
        # Ensure current workspace is searched before system path
        list(INSERT _source_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googlemock/src")
      endif()

      catkin_find_gmock_source("${_include_paths}" "${_source_paths}" gmock_found
                               gmock_base_dir gmock_include_dir gmock_lib_dir
                               gmock_libs gmock_main_libs gtest_found
                               gtest_base_dir gtest_include_dir gtest_lib_dir
                               gtest_libs gtest_main_libs)

      # If we found gmock, set it up to be built (which will also build gtest,
      # since it's included by gmock's CMakeLists.txt)
      if(gmock_found)
        set(GMOCK_FROM_SOURCE_FOUND ${gmock_found} CACHE INTERNAL "")
        set(GMOCK_FROM_SOURCE_INCLUDE_DIRS ${gmock_include_dir} CACHE INTERNAL "")
        set(GMOCK_FROM_SOURCE_LIBRARY_DIRS ${gmock_lib_dir} CACHE INTERNAL "")
        set(GMOCK_FROM_SOURCE_LIBRARIES ${gmock_libs} CACHE INTERNAL "")
        set(GMOCK_FROM_SOURCE_MAIN_LIBRARIES ${gmock_main_libs} CACHE INTERNAL "")

        # overwrite CMake install command to skip install rules for gtest targets
        # which have been added in version 1.8.0
        set(_CATKIN_SKIP_INSTALL_RULES TRUE)
        function(install)
          if(_CATKIN_SKIP_INSTALL_RULES)
            return()
          endif()
          _install(${ARGN})
        endfunction()
        add_subdirectory(${gmock_base_dir} ${gmock_lib_dir})
        set(_CATKIN_SKIP_INSTALL_RULES FALSE)

        set_target_properties(${gmock_libs} ${gmock_main_libs}
                              PROPERTIES EXCLUDE_FROM_ALL 1)

        message(STATUS "Found gmock sources under '${gmock_base_dir}': gmock will be built")
      else() # gmock not found-- look for system-installed gtest by itself
        set(_include_paths "/usr/include/gtest")
        if(CATKIN_TOPLEVEL)
          # search in the current workspace before
          list(INSERT _include_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googletest/include/gtest")
        endif()

        set(_source_paths "/usr/src/gtest/src")
        if(CATKIN_TOPLEVEL)
          # search in the current workspace before
          list(INSERT _source_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googletest/src")
        endif()

        catkin_find_gtest_source("${_include_paths}" "${_source_paths}" gtest_found
                               gtest_base_dir gtest_include_dir gtest_lib_dir
                               gtest_libs gtest_main_libs)
      endif()

      if(gtest_found)
        set(GTEST_FROM_SOURCE_FOUND ${gtest_found} CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_INCLUDE_DIRS ${gtest_include_dir} CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_LIBRARY_DIRS ${gtest_lib_dir} CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_LIBRARIES ${gtest_libs} CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_MAIN_LIBRARIES ${gtest_main_libs} CACHE INTERNAL "")


        #If gmock has been found, the gtest libraries have already been added
        if(NOT gmock_found)
          # overwrite CMake install command to skip install rules for gtest targets
          # which have been added in version 1.8.0
          set(_CATKIN_SKIP_INSTALL_RULES TRUE)
          function(install)
            if(_CATKIN_SKIP_INSTALL_RULES)
              return()
            endif()
            _install(${ARGN})
          endfunction()
          add_subdirectory(${gtest_base_dir} ${gtest_lib_dir})
          set(_CATKIN_SKIP_INSTALL_RULES FALSE)
          set_target_properties(${gtest_libs} ${gtest_main_libs}
                                PROPERTIES EXCLUDE_FROM_ALL 1)
        endif()

        message(STATUS "Found gtest sources under '${gtest_base_dir}': gtests will be built")
      else()
        if(CATKIN_TOPLEVEL)
          message(STATUS "gtest not found, C++ tests can not be built. Please install the gtest headers globally in your system or checkout gtest (by running 'git clone  https://github.com/google/googletest.git -b release-1.8.0' in the source space '${CMAKE_SOURCE_DIR}' of your workspace) to enable gtests")
        else()
          message(STATUS "gtest not found, C++ tests can not be built. Please install the gtest headers globally in your system to enable gtests")
        endif()
      endif()
    endif()

    if(GMOCK_FROM_SOURCE_FOUND)
      # set the same variables as find_package()
      # do NOT set in the cache since when using gmock/gtest from source
      # we must always add the subdirectory to have their targets defined
      set(GMOCK_FOUND ${GMOCK_FROM_SOURCE_FOUND})
      set(GMOCK_INCLUDE_DIRS ${GMOCK_FROM_SOURCE_INCLUDE_DIRS})
      set(GMOCK_LIBRARY_DIRS ${GMOCK_FROM_SOURCE_LIBRARY_DIRS})
      set(GMOCK_LIBRARIES ${GMOCK_FROM_SOURCE_LIBRARIES})
      set(GMOCK_MAIN_LIBRARIES ${GMOCK_FROM_SOURCE_MAIN_LIBRARIES})
      set(GMOCK_BOTH_LIBRARIES ${GMOCK_LIBRARIES} ${GMOCK_MAIN_LIBRARIES})
    endif()

    if(GTEST_FROM_SOURCE_FOUND)
      # set the same variables as find_package()
      # do NOT set in the cache since when using gtest from source
      # we must always add the subdirectory to have their targets defined
      set(GTEST_FOUND ${GTEST_FROM_SOURCE_FOUND})
      set(GTEST_INCLUDE_DIRS ${GTEST_FROM_SOURCE_INCLUDE_DIRS})
      set(GTEST_LIBRARY_DIRS ${GTEST_FROM_SOURCE_LIBRARY_DIRS})
      set(GTEST_LIBRARIES ${GTEST_FROM_SOURCE_LIBRARIES})
      set(GTEST_MAIN_LIBRARIES ${GTEST_FROM_SOURCE_MAIN_LIBRARIES})
      set(GTEST_BOTH_LIBRARIES ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
    endif()
  else()
    message(STATUS "Found gtest: gtests will be built")
    add_library(gtest SHARED IMPORTED)
    set_target_properties(gtest PROPERTIES IMPORTED_LOCATION "${GTEST_LIBRARIES}")
    add_library(gtest_main SHARED IMPORTED)
    set_target_properties(gtest_main PROPERTIES IMPORTED_LOCATION "${GTEST_MAIN_LIBRARIES}")
    set(GTEST_FOUND ${GTEST_FOUND} CACHE INTERNAL "")
    set(GTEST_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS} CACHE INTERNAL "")
    set(GTEST_LIBRARIES ${GTEST_LIBRARIES} CACHE INTERNAL "")
    set(GTEST_MAIN_LIBRARIES ${GTEST_MAIN_LIBRARIES} CACHE INTERNAL "")
    set(GTEST_BOTH_LIBRARIES ${GTEST_BOTH_LIBRARIES} CACHE INTERNAL "")
  endif()
else()
  message(STATUS "Found gmock: gmock and gtests will be built")
  add_library(gmock UNKNOWN IMPORTED)
  set_target_properties(gmock PROPERTIES IMPORTED_LOCATION "${GMOCK_LIBRARIES}")
  add_library(gmock_main UNKNOWN IMPORTED)
  set_target_properties(gmock_main PROPERTIES IMPORTED_LOCATION "${GMOCK_MAIN_LIBRARIES}")
  set(GMOCK_FOUND ${GMOCK_FOUND} CACHE INTERNAL "")
  set(GMOCK_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS} CACHE INTERNAL "")
  set(GMOCK_LIBRARIES ${GMOCK_LIBRARIES} CACHE INTERNAL "")
  set(GMOCK_MAIN_LIBRARIES ${GMOCK_MAIN_LIBRARIES} CACHE INTERNAL "")
  set(GMOCK_BOTH_LIBRARIES ${GMOCK_BOTH_LIBRARIES} CACHE INTERNAL "")

  set(GTEST_FOUND ${GMOCK_FOUND} CACHE INTERNAL "")
  set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS} CACHE INTERNAL "")
  set(GTEST_LIBRARY_DIRS ${GMOCK_LIBRARY_DIRS} CACHE INTERNAL "")
  set(GTEST_LIBRARIES ${GMOCK_LIBRARIES} CACHE INTERNAL "")
  set(GTEST_MAIN_LIBRARIES ${GMOCK_MAIN_LIBRARIES} CACHE INTERNAL "")
  set(GTEST_BOTH_LIBRARIES ${GMOCK_BOTH_LIBRARIES} CACHE INTERNAL "")
endif()

# For Visual C++, need to increase variadic template size to build gtest
if(GTEST_FOUND)
  if(WIN32)
    add_definitions(/D _VARIADIC_MAX=10)
  endif()
endif()
