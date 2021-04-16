#ifndef MULTI_GOAL_RRT_CONNECT_H
#define MULTI_GOAL_RRT_CONNECT_H

#include <vector>
#include <memory>
#include "planner_utils.h"
#include "CollisionDetector.h"

class MultiGoalRRTConnect {
public:
  MultiGoalRRTConnect(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size);
  ~MultiGoalRRTConnect() = default;

  std::vector<State> plan(State start_state, State goal_state);
  std::vector<State> planSimple(State start_state, State goal_state);

private:
  /* Get new state to add to tree
    if the distance from q1 to q2 is less than eps, this just returns q2
    else it returns the point eps distance along the vector from q1 to q2
  */
  std::pair<State,bool> get_new_state(State q1, State q2, double eps);

  // Pick a new state in the state space
  State random_valid_sample();

  // Remove shortcuttable states
  std::vector<State> shortcutPath(std::vector<State> plan_states);

  // Resample along path for smooth trajectory
  std::vector<State> interpolatePath(std::vector<State> plan_states);

  double vehicle_length_;
  double vehicle_width_;
  int x_size_;
  int y_size_;
  double cell_size_;

  std::shared_ptr<CollisionDetector> collision_detector_;

  // State lower bound
  State state_lo_;

  // State upper bound
  State state_hi_;
};

#endif