#include <iostream>

#include <b/foo.hpp>
#include <c/foo.hpp>

#if defined(_MSC_VER)
  #define OUTPUT_FUNCTION_NAME __FUNCSIG__
#else
  #define OUTPUT_FUNCTION_NAME __PRETTY_FUNCTION__
#endif

namespace d {
  void foo() { 
    b::foo();
    c::foo();
    std::cout << OUTPUT_FUNCTION_NAME << "\n";
  }
}
