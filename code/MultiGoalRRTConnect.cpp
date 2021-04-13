#include "MultiGoalRRTConnect.h"

MultiGoalRRTConnect::MultiGoalRRTConnect(double vehicle_length, double vehicle_width) {
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
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