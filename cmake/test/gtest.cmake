_generate_function_if_testing_is_disabled(
  "catkin_add_gtest"
  "catkin_add_gmock"
  "catkin_add_executable_with_gtest"
  "catkin_add_executable_with_gmock"
  "catkin_find_google_test_source")
_generate_function_if_no_cxx_language(
  "catkin_add_gtest"
  "catkin_add_gmock"
  "catkin_add_executable_with_gtest"
  "catkin_add_executable_with_gmock"
  "catkin_find_google_test_source")

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
#
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
#
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
endfunction()

# Internal function for finding gtest or gmock sources
function(_catkin_find_google_source type include_paths
  src_paths found base_dir include_dir lib_dir libs main_libs)
  # Find the gtest headers
  find_file(_${type}_INCLUDES "${type}.h"
            PATHS ${include_paths}
            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

  # Find the gtest sources
  find_file(_${type}_SOURCES "${type}.cc"
            PATHS ${src_paths}
            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

  # If we found gtest, set the variables accordingly
  if(_${type}_INCLUDES AND _${type}_SOURCES)
    get_filename_component(SOURCE_DIR ${_${type}_SOURCES} PATH)
    get_filename_component(BASE_DIR ${SOURCE_DIR} PATH)

    get_filename_component(INCLUDE_DIR ${_${type}_INCLUDES} PATH)
    get_filename_component(INCLUDE_DIR ${INCLUDE_DIR} PATH)

    set(${found} TRUE PARENT_SCOPE)
    set(${base_dir} ${BASE_DIR} PARENT_SCOPE)
    set(${include_dir} ${INCLUDE_DIR} PARENT_SCOPE)
    set(${lib_dir} ${CMAKE_BINARY_DIR}/${type} PARENT_SCOPE)
    set(${libs} "${type}" PARENT_SCOPE)
    set(${main_libs} "${type}_main" PARENT_SCOPE)
  endif()
endfunction()

# Find Google Test (GTest and optionally GMock) source-only install.
#
# Google recommends distributing GTest and GMock as source only, to be built with the same
# flags as that which is being tested.
#
# :param[in] gtest_path: Base path to search for gtest sources and includes, eg /usr
# :param[in] googletest_path: Base path to search for googletest-packaged gtest/gmock,
#                             eg, /usr/src/googletest
# :param[out] gtest_found: Whether or not GTest was found in the paths provided
# :param[out] gtest_include_dir: The include path to access GTest's headers
# :param[out] gtest_lib_dir: The library path to access GTest's libraries
# :param[out] gtest_libs: GTest's libraries
# :param[out] gtest_main_libs: GTest's main libraries
# :param[out] gmock_found: Whether or not GMock was found in the paths provided
# :param[out] gmock_include_dir: The include path to access GMock's headers
# :param[out] gmock_lib_dir: The library path to access GMock's libraries
# :param[out] gmock_libs: GMock's libraries
# :param[out] gmock_main_libs: GMock's main libraries
# :param[out] base_dir: The base directory containing Google Test and/or GMock CMakeLists.txt
#
function(catkin_find_google_test_source gtest_path googletest_path
  gtest_found gtest_include_dir gtest_lib_dir gtest_libs gtest_main_libs
  gmock_found gmock_include_dir gmock_lib_dir gmock_libs gmock_main_libs
  base_dir
)
  # Path to gtest from the libgtest-dev Debian package.
  set(_gtest_include_paths "${gtest_path}/include/gtest")
  set(_gtest_source_paths "${gtest_path}/src/gtest/src")

  # Path to gtest from the googletest Debian package.
  list(APPEND _gtest_include_paths "${googletest_path}/googletest/include/gtest")
  list(APPEND _gtest_source_paths "${googletest_path}/googletest/src")

  # Path to gtest from the gtest Arch Linux package. Header location already included
  # list(APPEND _gtest_include_paths "${gtest_path}/include/gtest")
  list(APPEND _gtest_source_paths "${googletest_path}/src")

  if(CATKIN_TOPLEVEL)
    # Ensure current workspace is searched before system path
    list(INSERT _gtest_include_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googletest/include/gtest")
    list(INSERT _gtest_source_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googletest/src")
  endif()

  _catkin_find_google_source("gtest" "${_gtest_include_paths}"
                             "${_gtest_source_paths}" _gtest_found
                             _gtest_base_dir _gtest_include_dir _gtest_lib_dir
                             _gtest_libs _gtest_main_libs)
  set(${gtest_found} ${_gtest_found} PARENT_SCOPE)
  set(${gtest_base_dir} ${_gtest_base_dir} PARENT_SCOPE)
  set(${gtest_include_dir} ${_gtest_include_dir} PARENT_SCOPE)
  set(${gtest_lib_dir} ${_gtest_lib_dir} PARENT_SCOPE)
  set(${gtest_libs} ${_gtest_libs} PARENT_SCOPE)
  set(${gtest_main_libs} ${_gtest_main_libs} PARENT_SCOPE)
  set(${base_dir} ${_gtest_base_dir} PARENT_SCOPE)

  # Path to gmock from the google-mock Debian package before v1.8.0.
  set(_gmock_include_paths "${gtest_path}/include/gmock")
  set(_gmock_source_paths "${gtest_path}/src/gmock/src")

  # Path to gmock from the googletest Debian package.
  list(APPEND _gmock_include_paths "${googletest_path}/googlemock/include/gmock")
  list(APPEND _gmock_source_paths "${googletest_path}/googlemock/src")

  if(CATKIN_TOPLEVEL)
    # Ensure current workspace is searched before system path
    list(INSERT _gmock_include_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googlemock/include/gmock")
    list(INSERT _gmock_source_paths 0 "${CMAKE_SOURCE_DIR}/googletest/googlemock/src")
  endif()

  _catkin_find_google_source("gmock" "${_gmock_include_paths}"
                             "${_gmock_source_paths}" _gmock_found
                             _gmock_base_dir _gmock_include_dir _gmock_lib_dir
                             _gmock_libs _gmock_main_libs)
  if(_gmock_found)
    set(${gmock_found} ${_gmock_found} PARENT_SCOPE)
    set(${gmock_base_dir} ${_gmock_base_dir} PARENT_SCOPE)
    set(${gmock_include_dir} ${_gmock_include_dir} PARENT_SCOPE)
    set(${gmock_lib_dir} ${_gmock_lib_dir} PARENT_SCOPE)
    set(${gmock_libs} ${_gmock_libs} PARENT_SCOPE)
    set(${gmock_main_libs} ${_gmock_main_libs} PARENT_SCOPE)

    #Overwrite gtest base_dir with gmock's
    set(${base_dir} ${_gmock_base_dir} PARENT_SCOPE)
  endif()

  # In googletest 1.8, gmock and gtest were merged inside googletest
  # In this case, including gmock builds gmock twice, so instead we need to include googletest
  # which will include gtest and gmock once
  if(_gtest_found)
    get_filename_component(_gtest_base_dir_realpath ${_gtest_base_dir} REALPATH)
    get_filename_component(_global_base_dir ${_gtest_base_dir_realpath} PATH)
    if(EXISTS "${_global_base_dir}/CMakeLists.txt")
      set(${base_dir} ${_global_base_dir} PARENT_SCOPE)
    endif()
  endif()
endfunction()

find_package(GMock QUIET)
# the GMockConfig.cmake from the Debian package cmake-extras provides all targets
if(TARGET gtest AND TARGET gtest_main AND TARGET gmock AND TARGET gmock_main)
  set(GMOCK_FOUND TRUE)
  set(GTEST_FOUND TRUE)

  # If we build these from source in a previous catkin (sub)project,
  # define all variables from their FROM_SOURCE counterparts (again).
  # As these variables are (explicitly!) not cached, they become reset on a new project.
  if(GMOCK_FROM_SOURCE_FOUND)
    set(GMOCK_INCLUDE_DIRS ${GMOCK_FROM_SOURCE_INCLUDE_DIRS})
    set(GMOCK_LIBRARY_DIRS ${GMOCK_FROM_SOURCE_LIBRARY_DIRS})
    set(GMOCK_LIBRARIES ${GMOCK_FROM_SOURCE_LIBRARIES})
    set(GMOCK_MAIN_LIBRARIES ${GMOCK_FROM_SOURCE_MAIN_LIBRARIES})
    set(GMOCK_BOTH_LIBRARIES ${GMOCK_LIBRARIES} ${GMOCK_MAIN_LIBRARIES})
  endif()
  if(GTEST_FROM_SOURCE_FOUND)
    set(GTEST_INCLUDE_DIRS ${GTEST_FROM_SOURCE_INCLUDE_DIRS})
    set(GTEST_LIBRARY_DIRS ${GTEST_FROM_SOURCE_LIBRARY_DIRS})
    set(GTEST_LIBRARIES ${GTEST_FROM_SOURCE_LIBRARIES})
    set(GTEST_MAIN_LIBRARIES ${GTEST_FROM_SOURCE_MAIN_LIBRARIES})
    set(GTEST_BOTH_LIBRARIES ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
  endif()
else()
  find_package(GTest QUIET)
endif()
if(NOT GMOCK_FOUND OR NOT GTEST_FOUND)
  # If we find one but not the other, see if we can get both from source
  # only add gmock/gtest directory once per workspace
  if(NOT TARGET gtest AND NOT TARGET gmock)
    # Path to base of legacy libgtest-dev and google-mock packages.
    set(_gtest_path "/usr")

    # Path to base of new googletest package, which includes both gtest and gmock.
    set(_googletest_path "/usr/src/googletest")

    catkin_find_google_test_source("${_gtest_path}" "${_googletest_path}" gtest_found
                                   gtest_include_dir gtest_lib_dir gtest_libs gtest_main_libs
                                   gmock_found gmock_include_dir gmock_lib_dir gmock_libs
                                   gmock_main_libs base_dir)
    if (gtest_found AND gmock_found)
      if(GMOCK_FOUND OR GTEST_FOUND)
        message(STATUS "Forcing gtest/gmock from source, though one was otherwise available.")
      endif()
      set(FORCE_GTEST_GMOCK_FROM_SOURCE TRUE)
    endif()
  endif()
endif()

if(FORCE_GTEST_GMOCK_FROM_SOURCE OR (NOT GMOCK_FOUND AND NOT GTEST_FOUND))
  if(gtest_found)
    set(GTEST_FROM_SOURCE_FOUND ${gtest_found} CACHE INTERNAL "")
    set(GTEST_FROM_SOURCE_INCLUDE_DIRS ${gtest_include_dir} CACHE INTERNAL "")
    set(GTEST_FROM_SOURCE_LIBRARY_DIRS ${gtest_lib_dir} CACHE INTERNAL "")
    set(GTEST_FROM_SOURCE_LIBRARIES ${gtest_libs} CACHE INTERNAL "")
    set(GTEST_FROM_SOURCE_MAIN_LIBRARIES ${gtest_main_libs} CACHE INTERNAL "")
    message(STATUS "Found gtest sources under '${base_dir}': gtests will be built")
  endif()
  if(gmock_found)
    set(GMOCK_FROM_SOURCE_FOUND ${gmock_found} CACHE INTERNAL "")
    set(GMOCK_FROM_SOURCE_INCLUDE_DIRS ${gmock_include_dir} CACHE INTERNAL "")
    set(GMOCK_FROM_SOURCE_LIBRARY_DIRS ${gmock_lib_dir} CACHE INTERNAL "")
    set(GMOCK_FROM_SOURCE_LIBRARIES ${gmock_libs} CACHE INTERNAL "")
    set(GMOCK_FROM_SOURCE_MAIN_LIBRARIES ${gmock_main_libs} CACHE INTERNAL "")
    message(STATUS "Found gmock sources under '${base_dir}': gmock will be built")
  endif()
  if(base_dir)
    # overwrite CMake install command to skip install rules for gtest targets
    # which have been added in version 1.8.0
    _use_custom_install()
    set(_CATKIN_SKIP_INSTALL_RULES TRUE)
    add_subdirectory(${base_dir} ${gtest_lib_dir})
    set(_CATKIN_SKIP_INSTALL_RULES FALSE)
    set_target_properties(${gtest_libs} ${gtest_main_libs}
                          PROPERTIES EXCLUDE_FROM_ALL 1)
    if(gmock_found)
      set_target_properties(${gmock_libs} ${gmock_main_libs}
                            PROPERTIES EXCLUDE_FROM_ALL 1)
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
  if(GMOCK_FOUND)
    message(STATUS "Found gmock: gmock and gtests will be built")
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
  elseif(GTEST_FOUND)
    message(STATUS "Found gtest: gtests will be built")
    set(GTEST_FOUND ${GTEST_FOUND} CACHE INTERNAL "")
    set(GTEST_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS} CACHE INTERNAL "")
    set(GTEST_LIBRARIES ${GTEST_LIBRARIES} CACHE INTERNAL "")
    set(GTEST_MAIN_LIBRARIES ${GTEST_MAIN_LIBRARIES} CACHE INTERNAL "")
    set(GTEST_BOTH_LIBRARIES ${GTEST_BOTH_LIBRARIES} CACHE INTERNAL "")
  endif()
endif()

if(NOT GTEST_FOUND)
  if(CATKIN_TOPLEVEL)
    message(STATUS "gtest not found, C++ tests can not be built. Please install the gtest headers globally in your system or checkout gtest (by running 'git clone  https://github.com/google/googletest.git -b release-1.8.0' in the source space '${CMAKE_SOURCE_DIR}' of your workspace) to enable gtests")
  else()
    message(STATUS "gtest not found, C++ tests can not be built. Please install the gtest headers globally in your system to enable gtests")
  endif()
endif()

if(GMOCK_FOUND AND NOT TARGET gmock)
  add_library(gmock UNKNOWN IMPORTED)
  set_target_properties(gmock PROPERTIES IMPORTED_LOCATION "${GMOCK_LIBRARIES}")
endif()
if(GMOCK_FOUND AND NOT TARGET gmock_main)
  add_library(gmock_main UNKNOWN IMPORTED)
  set_target_properties(gmock_main PROPERTIES IMPORTED_LOCATION "${GMOCK_MAIN_LIBRARIES}")
endif()
if(GTEST_FOUND AND NOT TARGET gtest)
  add_library(gtest SHARED IMPORTED)
  set_target_properties(gtest PROPERTIES IMPORTED_LOCATION "${GTEST_LIBRARIES}")
endif()
if(GTEST_FOUND AND NOT TARGET gtest_main)
  add_library(gtest_main SHARED IMPORTED)
  set_target_properties(gtest_main PROPERTIES IMPORTED_LOCATION "${GTEST_MAIN_LIBRARIES}")
endif()

# For Visual C++, need to increase variadic template size to build gtest
if(GTEST_FOUND)
  if(WIN32)
    add_definitions(/D _VARIADIC_MAX=10)
  endif()
endif()
