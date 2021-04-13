#include "MultiGoalRRTConnect.h"
#include <iostream>

MultiGoalRRTConnect::MultiGoalRRTConnect(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size) {
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
  x_size_ = x_size;
  y_size_ = y_size;

  occupancy_grid_ = new bool[x_size_ * y_size_];
  memcpy(occupancy_grid_, occupancy_grid, x_size_ * y_size_ * sizeof(bool));
  cell_size_ = cell_size;
}

MultiGoalRRTConnect::~MultiGoalRRTConnect() {
  delete[] occupancy_grid_;
}

std::vector<State> MultiGoalRRTConnect::plan(State start_state, State goal_state) {

	// Arbitrary plan to test mex bindings
  int plan_length = 200;
  std::vector<State> plan(plan_length);

  // Just interpolate from start to goal
  for (int i = 0; i < plan.size(); ++i) {
    plan.at(i) = get_intermediate_state(start_state,goal_state,(double)i/(plan_length-1));
  }

  return plan;
}

// True means collision, false means free
bool MultiGoalRRTConnect::checkCollision(State state) {

  // Remember to map x,y by cell size to get the correct index in the map
  int index_test = GETMAPINDEX(3,2,x_size_,y_size_);
  return false;
}