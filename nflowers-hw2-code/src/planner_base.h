#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include <vector>
#include <string.h>
#include <memory>

#include "helpers.h"
#include "tree.h"

class PlannerBase{
public:
  PlannerBase(int dof, double *map, int x_size, int y_size);
  
  ~PlannerBase();

  virtual void plan(State q_init, State q_goal, double*** plan, int* plan_length) {}

  void transferRefinedPlan(double*** plan, int* plan_length, std::vector<State> plan_states);

  virtual int getVertices() {}

protected:

  std::vector<State> shortcutPath(std::vector<State> plan_states);

  std::vector<State> interpolatePath(std::vector<State> plan_states);

  std::vector<double> random_valid_sample();

  bool checkCollision(State q1, State q2, int N);

  std::pair<State,bool> getNewState(State q1, State q2, double eps);

  State getIntermediateState(State q1, State q2, double r);

  int dof_;
  double* map_;
  int x_size_;
  int y_size_;
  
  
};

#endif