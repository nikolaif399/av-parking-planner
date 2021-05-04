#include "code/reeds_shepp.h"
#include <iostream>
#include <chrono>
using namespace std;


int main(int argc, char** argv) {
  using namespace std::chrono;
  std::cout << "Test started!" << std::endl;

  double start[3] = {0,0,0};
  double goal[3] = {5,0,0};

  ReedsSheppStateSpace rs = ReedsSheppStateSpace(1);

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  ReedsSheppStateSpace::ReedsSheppPath rspath = rs.reedsShepp(start,goal);
  
  // double interpolated_state[3];

  double r = 0;
  double interpolated_state[3];
  
  
  while (r < rspath.length()) {
    rs.interpolate(start,rspath, r, interpolated_state);
    r += 0.1;
  }
  
  printf("Interpolated state: [%.2f, %.2f, %.2f]", interpolated_state[0], interpolated_state[1], interpolated_state[2]);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();

  duration<double> time = duration_cast<duration<double> >(t2 - t1);
  
// To get the value of duration use the count()
// member function on the duration object
  std::cout << "Reeds shepp function takes " << time.count() * 1e6 << " microseconds." << std::endl;
  std::cout << "Distance: " << rspath.length() << std::endl;
  std::cout << "Test completed!" << std::endl;
  return 0;
}
