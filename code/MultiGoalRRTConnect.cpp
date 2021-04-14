#include "MultiGoalRRTConnect.h"
#include <iostream>

MultiGoalRRTConnect::MultiGoalRRTConnect(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size) {
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
  x_size_ = x_size;
  y_size_ = y_size;
  cell_size_ = cell_size;

  collision_detector_ = std::make_shared<CollisionDetector>(vehicle_length, vehicle_width, x_size, y_size, occupancy_grid, cell_size);
}

std::vector<State> MultiGoalRRTConnect::plan(State start_state, State goal_state) {

	// Arbitrary plan to test mex bindings
  int plan_length = 200;
  std::vector<State> plan(plan_length);

  collision_detector_->checkCollision(start_state);

  // Just interpolate from start to goal
  for (int i = 0; i < plan.size(); ++i) {
    plan.at(i) = get_intermediate_state(start_state,goal_state,(double)i/(plan_length-1));
  }

  return plan;
}