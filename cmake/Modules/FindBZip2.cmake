# - Try to find BZip2
# Once done this will define
#
#  BZIP2_FOUND - system has BZip2
#  BZIP2_INCLUDE_DIR - the BZip2 include directory
#  BZIP2_LIBRARIES - Link these to use BZip2
#  BZIP2_NEED_PREFIX - this is set if the functions are prefixed with BZ2_
#
# This differs from the original script by introducing 
# BZIP2_ROOT to help it find the package when it is installed
# on a system with no standard search directories (windoze).

FIND_PATH(BZIP2_INCLUDE_DIR bzlib.h PATHS ${BZIP2_ROOT}/include )

FIND_LIBRARY(BZIP2_LIBRARIES NAMES bz2 bzip2 PATHS ${BZIP2_ROOT}/lib)

# handle the QUIETLY and REQUIRED arguments and set BZip2_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(BZip2 DEFAULT_MSG BZIP2_LIBRARIES BZIP2_INCLUDE_DIR)

IF (BZIP2_FOUND)
   INCLUDE(CheckLibraryExists)
   CHECK_LIBRARY_EXISTS(${BZIP2_LIBRARIES} BZ2_bzCompressInit "" BZIP2_NEED_PREFIX)
ENDIF (BZIP2_FOUND)

MARK_AS_ADVANCED(BZIP2_INCLUDE_DIR BZIP2_LIBRARIES)

