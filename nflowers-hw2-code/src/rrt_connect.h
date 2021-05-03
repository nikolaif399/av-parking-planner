#ifndef RRT_CONNECT_PLANNER_H
#define RRT_CONNECT_PLANNER_H

#include "planner_base.h"

class PlannerRRTConnect : public PlannerBase{
public:
  PlannerRRTConnect(int dof, double *map, int x_size, int y_size, int k, double eps);

  void plan(State q_init, State q_goal, double*** plan, int* plan_length) override;

  int getVertices() override;

  std::shared_ptr<Tree> tree_start_;

  std::shared_ptr<Tree> tree_goal_;
private:

  void extend(State q_sample, std::shared_ptr<Tree> tree_extending_towards, int& ret_flag, int& new_index);

  void connect(State q_new, int& ret_flag, int& new_index);

  std::shared_ptr<Tree> cur_tree_,other_tree_;
  
  bool cur_equal_start_;

  int k_;

  double eps_;
};


#endif