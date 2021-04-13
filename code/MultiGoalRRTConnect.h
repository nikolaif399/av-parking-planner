#ifndef MULTI_GOAL_RRT_CONNECT_H
#define MULTI_GOAL_RRT_CONNECT_H

#include <vector>
#include "planner_utils.h"

class MultiGoalRRTConnect {
public:
  MultiGoalRRTConnect(double vehicle_length, double vehicle_width);

  std::vector<State> plan(State start_state, State goal_state);

private:
  double vehicle_length_;
  double vehicle_width_;
};

#endif