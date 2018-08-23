#include <iostream>

#include <a/foo.hpp>

#if defined(_MSC_VER)
  #define OUTPUT_FUNCTION_NAME __FUNCSIG__
#else
  #define OUTPUT_FUNCTION_NAME __PRETTY_FUNCTION__
#endif

namespace c {
  void foo() { 
    a::foo();
    std::cout << OUTPUT_FUNCTION_NAME << "\n";
  }
}
