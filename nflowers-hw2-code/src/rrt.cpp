#include "rrt.h"

#define REACHED 0
#define ADVANCED 1
#define TRAPPED 2

PlannerRRT::PlannerRRT(int dof, double *map, int x_size, int y_size, int k, double eps, double goal_prob) : 
  PlannerBase(dof, map, x_size, y_size){

  k_ = k;
  eps_ = eps;
  goal_prob_ = goal_prob;
  return_on_success_ = true;

  tree_start_ = std::make_shared<Tree>(dof);
}

int PlannerRRT::getVertices() {
  return tree_start_->getSize();
}

void PlannerRRT::plan(State q_init, State q_goal, double*** plan, int* plan_length) {
  int start_ind = tree_start_->addVertex(q_init);

  for (int i = 0; i < k_; ++i) {
    
    // If rand is less than goal threshold, pick goal point, else pick random point 
    bool connect2goal = GetRand(0,1) < goal_prob_;
    if (goal_index_ != -1) connect2goal = false;
    State q_sample = connect2goal ? q_goal : this->random_valid_sample();

    // Extend
    int extend_ret,new_index;
    this->extend(q_sample,connect2goal,extend_ret,new_index);

    if (connect2goal && extend_ret == REACHED) {
      //printf("Reached goal!\n");
      goal_index_ = new_index;
      if (return_on_success_) break;
    }

    /*if (goal_index_ != -1 && i % 500 == 0) {
      printf("Iteration %d: goal index %d, goal cost %f\n", i, goal_index_, tree_start_->getCost(goal_index_));
    }*/
  }
  
  if (goal_index_ != -1) {
    std::vector<State> plan_states = this->tree_start_->getPath(start_ind, goal_index_);
    printf("Raw plan size: %zu\n", plan_states.size());
    
    plan_states = this->shortcutPath(plan_states);
    printf("Plan size after shortcutting: %zu\n", plan_states.size());

    plan_states = this->interpolatePath(plan_states);
    printf("Plan size after interpolating: %zu\n", plan_states.size());

    printf("Plan cost: %f\n", tree_start_->getCost(goal_index_));

    tree_start_->saveTree(x_size_,"start_tree.txt");

    this->transferRefinedPlan(plan,plan_length,plan_states);
  }
}

void PlannerRRT::extend(State q_sample, bool connect2goal, int& ret_flag, int& new_index) {
  // Get nearest neighbor to qrand
  int nearest_index;
  if (connect2goal) {
    nearest_index = this->tree_start_->getNearestVertexExcluding(q_sample, goal_conn_failed_);
  }
  else {
    nearest_index = this->tree_start_->getNearestVertex(q_sample);
  }

  // Couldn't find new point to extend towards goal
  if (nearest_index == -1) {
    ret_flag = TRAPPED;
    new_index = -1;
    return;
  }
  
  State q_nearest = this->tree_start_->getState(nearest_index);
  std::pair<State,bool> new_state = this->getNewState(q_nearest,q_sample,eps_);
  State q_new = new_state.first;

  // Check for collision using twenty intermediate states
  
  if(checkCollision(q_nearest, q_new, 20)) {
    new_index = this->tree_start_->addVertex(q_new);
    this->tree_start_->addEdge(nearest_index, new_index);
    if (connect2goal && !new_state.second) {
      goal_conn_failed_.insert(nearest_index);
    }
    ret_flag = new_state.second ? REACHED : ADVANCED;
    return;
  }

  if (connect2goal) {
    goal_conn_failed_.insert(nearest_index);
  }

  ret_flag = TRAPPED;
  new_index = -1;
}