#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

#include "planner_base.h"
#include "graph.h"

class PlannerPRM : public PlannerBase{
public:
  PlannerPRM(int dof, double *map, int x_size, int y_size, int N, double neighborhood);

  void plan(State q_init, State q_goal, double*** plan, int* plan_length) override;

  int getVertices() override;

private:

  std::shared_ptr<Graph> graph_;
  
  int N_;

  double neighborhood_;
};


#endif 
