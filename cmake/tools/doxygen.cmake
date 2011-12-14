#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

##
# doxygen(<TARGET_NAME> <SEARCH_DIRS>)
# TARGET_NAME -> The cmake target to create.
# SEARCH_DIRS -> a CMake List of directories to search for doxygenated files.
#
macro(doxygen TARGET_NAME SEARCH_DIRS)
  #doxygen based docs
  set(DOC_SEARCH_DIRS ${SEARCH_DIRS}
  )
  foreach(dir ${DOC_SEARCH_DIRS})
    file(GLOB_RECURSE _doc_sources ${dir}/*)
    list(APPEND doc_sources ${_doc_sources})
  endforeach()

  string(REPLACE ";" " " doc_sources "${doc_sources}")

  configure_file(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

  add_custom_target(${TARGET_NAME}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )

  add_custom_command(TARGET ${TARGET_NAME}
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_dependencies(doc ${TARGET_NAME})
endmacro()

macro(find_sphinx)
  find_program(SPHINX_BUILD sphinx-build)
  if(SPHINX_BUILD)
    set(REQUIRED_SPHINX_VERSION "1.0.7")
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sphinx;print sphinx.__version__"
      OUTPUT_VARIABLE SPHINX_VERSION
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
    if("${SPHINX_VERSION}" VERSION_LESS ${REQUIRED_SPHINX_VERSION})
      MESSAGE(WARNING "You version of sphinx (http://sphinx.pocoo.org) is ${SPHINX_VERSION}, required ${REQUIRED_SPHINX_VERSION}")
      if (UNIX)
        MESSAGE(WARNING "You may be able to update with 'easy_install -U sphinx'")
      endif()
    endif()
  endif()
endmacro()

