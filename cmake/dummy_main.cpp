#include <iostream>

#define STRINGIZE(X) #X
int main(int, char**)
{
  std::cout << "DUMMY MAIN PROVIDED BY CATKIN: FIXME.\n"
            << "Target: " << STRINGIZE(CATKIN_DUMMY_TARGET)
            << "\n";
}


