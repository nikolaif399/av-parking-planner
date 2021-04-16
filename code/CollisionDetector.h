#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <memory.h>
#include <math.h>
#include <limits> // for infinity
#include <algorithm> // for min
#include "planner_utils.h"

class CollisionDetector {
public:
  CollisionDetector(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size);
  ~CollisionDetector();

  // True means collision, false means free
  bool checkCollision(State state);

  // True means collision interpolating from q1 to q2, false means free
  bool checkCollisionLine(State q1, State q2, int N);

  static std::vector<std::pair<int,int>> raster_line(double x0, double y0, double x1, double y1);
private:

  void pointToGridIndices(double x, double y, int &x_ind, int &y_ind);

  double vehicle_length_;
  double vehicle_width_;
  int x_size_;
  int y_size_;
  bool* occupancy_grid_;
  double cell_size_ = 1;
};

#endif