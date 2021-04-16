#ifndef RRT_CONNECT_PLANNER_H
#define RRT_CONNECT_PLANNER_H

#include <memory>
#include "tree.h"

class PlannerRRTConnect{
public:
  PlannerRRTConnect(int dof, double *map, int x_size, int y_size, int k, double eps);

  void plan(State q_init, State q_goal, double*** plan, int* plan_length) override;

  int getNumVertices();

  std::shared_ptr<Tree> tree_start_;

  std::shared_ptr<Tree> tree_goal_;

private:

  // RRT-Connect Extend Function from class notes
  void extend(State q_sample, std::shared_ptr<Tree> tree_extending_towards, int& ret_flag, int& new_index);

  // RRT-Connect Connect Function from class notes
  void connect(State q_new, int& ret_flag, int& new_index);

  // Remove shortcuttable states
  std::vector<State> shortcutPath(std::vector<State> plan_states);

  // Resample along path for smooth trajectory
  std::vector<State> interpolatePath(std::vector<State> plan_states);

  // Pick a new state in the state space
  State random_valid_sample();

  /* Get intermediate state between q1 and q2 (r is the ratio along that vector
  r = 0 will return q1
  r = 0.5 will return halfway between q1 and q2
  r = 1 with return q2, etc
  */
  State get_intermediate_state(State q1, State q2, double r);

  /* Get new state to add to tree
    if the distance from q1 to q2 is less than eps, this just returns q2
    else it returns the point eps distance along the vector from q1 to q2
  */
  std::pair<State,bool> getNewState(State q1, State q2, double eps);
  
  std::shared_ptr<Tree> cur_tree_,other_tree_;
  bool cur_equal_start_;
  int k_;
  double eps_;
};


#endif