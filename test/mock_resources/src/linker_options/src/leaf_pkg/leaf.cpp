#include <iostream>
#include <root_pkg/root.hpp>

int main()
{
  std::cout << "leaf_pkg calling ";
  root_pkg::func();
  return 0;
}
