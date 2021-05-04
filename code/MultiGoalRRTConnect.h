#ifndef MULTI_GOAL_RRT_CONNECT_H
#define MULTI_GOAL_RRT_CONNECT_H

#include <vector>
#include <memory>
#include "planner_utils.h"
#include "CollisionDetector.h"
#include "tree.h"
#include "reeds_shepp.h"

#define REACHED 0
#define ADVANCED 1
#define TRAPPED 2

class MultiGoalRRTConnect {

public:
  MultiGoalRRTConnect(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size, int k, double eps);
  ~MultiGoalRRTConnect() = default;

  std::vector<State> plan(State start_state, std::vector<State> goal_states);

  std::shared_ptr<Tree> start_tree_;
  std::vector<std::shared_ptr<Tree>> goal_trees_;

private:
  /* Primary extend function
  tries to extend tree_extending_from towards q_sample
  returns success flag as well as index of new state in tree if successful
  */
  void extend(State q_sample, std::shared_ptr<Tree> tree_extending_from, int& ret_flag, int& new_index);

  // RRT-Connect Connect Function from class notes
  void connect(State q_new, int& ret_flag, int& new_index);

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

  // Assemble final trajectory through trees
  std::vector<State> getPath(int start_ind, int start_ind_mid, int goal_ind, int goal_ind_mid, std::shared_ptr<Tree> goal_tree);

  double vehicle_length_;
  double vehicle_width_;
  int x_size_;
  int y_size_;
  double cell_size_;
  
  // Number of RRT iterations
  int k_;

  // RRT Extent
  double eps_;

  // Temporary, will need updated for multi goals
  std::shared_ptr<Tree> cur_tree_,other_tree_;
  bool cur_equal_start_;

  // State lower bound
  State state_lo_;

  // State upper bound
  State state_hi_;

  // Collision detector
  std::shared_ptr<CollisionDetector> collision_detector_;

  // Reeds shepp state connector
  std::shared_ptr<ReedsSheppStateSpace> state_connector_;
};

#endif