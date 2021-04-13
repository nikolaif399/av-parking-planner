#ifndef MULTI_GOAL_RRT_CONNECT_H
#define MULTI_GOAL_RRT_CONNECT_H

#include <vector>
#include <memory.h>
#include "planner_utils.h"


class MultiGoalRRTConnect {
public:
  MultiGoalRRTConnect(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size);
  ~MultiGoalRRTConnect();

  std::vector<State> plan(State start_state, State goal_state);

private:

  bool checkCollision(State state);

  double vehicle_length_;
  double vehicle_width_;
  int x_size_;
  int y_size_;
  bool* occupancy_grid_;
  double cell_size_;
};

#endif