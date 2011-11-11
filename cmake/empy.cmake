find_program(EMPY_EXECUTABLE empy)
if (NOT EMPY_EXECUTABLE)
  message(FATAL_ERROR "Unable to find executable 'empy'... try installing package 'python-empy'")
endif()

