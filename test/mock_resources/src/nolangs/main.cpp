#include <iostream>

#if defined(_MSC_VER)
  #define OUTPUT_FUNCTION_NAME __FUNCSIG__
#else
  #define OUTPUT_FUNCTION_NAME __PRETTY_FUNCTION__
#endif

int main(int, char**)  {
  std::cout << OUTPUT_FUNCTION_NAME << "\n";
}
