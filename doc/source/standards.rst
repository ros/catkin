CMake coding standards
----------------------

* use REQUIRED on your calls to find_package if they aren't actually
  optional (i.e. you're not going to check ``thing_FOUND`` and
  enable/disable features)

