#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include "planner_base.h"

class PlannerRRT : public PlannerBase{
public:
  // K is number of iterations
  
  PlannerRRT(int dof, double *map, int x_size, int y_size, int k, double eps, double goal_prob);

  void plan(State q_init, State q_goal, double*** plan, int* plan_length) override;

  int getVertices() override;

  std::shared_ptr<Tree> tree_start_;
protected:

  virtual void extend(State q_sample, bool connect2goal, int& ret_flag, int& new_index);

  int k_;
  double eps_;
  double goal_prob_;
  int goal_index_ = -1;

  bool return_on_success_;

  std::unordered_set<int> goal_conn_failed_;
};

#endif