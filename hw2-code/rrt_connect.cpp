#include "rrt_connect.h"

#define REACHED 0
#define ADVANCED 1
#define TRAPPED 2

PlannerRRTConnect::PlannerRRTConnect(int dof, double *map, int x_size, int y_size, int k, double eps) :
  PlannerBase(dof, map, x_size, y_size){
  
  k_ = k;
  eps_ = eps;

  tree_start_ = std::make_shared<Tree>(dof);
  tree_goal_ = std::make_shared<Tree>(dof);
  cur_tree_ = tree_start_;
  cur_equal_start_ = true;
  other_tree_ = tree_goal_;
}

int PlannerRRTConnect::getNumVertices() {
  return tree_start_->getSize() + tree_goal_->getSize();
}

void PlannerRRTConnect::plan(State q_init, State q_goal, double*** plan, int* plan_length) {
  // Add start state to start tree
  int start_ind = tree_start_->addVertex(q_init);

  // Add goal state to goal tree
  int goal_ind = tree_goal_->addVertex(q_goal);

  // RRT-Connect iterations
  for (int i = 0; i < k_; ++i) { 

    // Draw a random valid sample from the state space
    State q_sample = this->random_valid_sample();

    // Try to extend the current tree towards to the new sample
    int ret_flag,new_index_extend;
    this->extend(q_sample, cur_tree_, ret_flag, new_index_extend);

    // If not trapped, try to connect to other tree
    if (ret_flag != TRAPPED) {
      int new_index_connect;
      State q_new = cur_tree_->getState(new_index_extend);
      this->connect(q_new, ret_flag, new_index_connect);

      // If connect reached, we have a path from start to goal!
      // Compute the index of the midpoint state in both trees
      if (ret_flag == REACHED) {
        int start_ind_mid,goal_ind_mid;
        if (cur_equal_start_) {
          start_ind_mid = new_index_extend;
          goal_ind_mid = new_index_connect;
        }
        else {
          start_ind_mid = new_index_connect;
          goal_ind_mid = new_index_extend;
        }
        printf("Reached goal through start tree index %d and goal tree index %d!\n", start_ind_mid, goal_ind_mid);
        
        // Compute plan from start to midpoint through start tree
        std::vector<State> plan_states_start = this->tree_start_->getPath(start_ind, start_ind_mid);
        
        // Compute plan from goal to midpoint through goal tree
        std::vector<State> plan_states_goal = this->tree_goal_->getPath(goal_ind, goal_ind_mid);
        
        // Flip path through goal tree
        std::reverse(plan_states_goal.begin(), plan_states_goal.end());

        // Concatenate start -> midpoint path with midpoint -> goal path
        std::vector<State> plan_states;
        plan_states.insert(plan_states.begin(), plan_states_start.begin(), plan_states_start.end());
        plan_states.insert(plan_states.end(), plan_states_goal.begin(), plan_states_goal.end());
        printf("Raw plan size: %zu\n", plan_states.size());
        
        // Remove shortcuttable states
        plan_states = this->shortcutPath(plan_states);
        printf("Plan size after shortcutting: %zu\n", plan_states.size());

        // Interpolate along path to get smooth trajectory
        plan_states = this->interpolatePath(plan_states);
        printf("Plan size after interpolating: %zu\n", plan_states.size());

        //this->transferRefinedPlan(plan,plan_length,plan_states);

        //tree_start_->saveTree(x_size_,"start_tree.txt");
        //tree_goal_->saveTree(x_size_,"goal_tree.txt");

        // Found plan, exit
        return;
      }
    }
    
    // Exchange goal tree and start tree
    std::shared_ptr<Tree> temp = cur_tree_;
    cur_tree_ = other_tree_;
    other_tree_ = temp;
    cur_equal_start_ = !cur_equal_start_;
  }
}

void PlannerRRTConnect::extend(State q_sample, std::shared_ptr<Tree> tree_extending_from, int& ret_flag, int& new_index) {
  // Get nearest neighbor to qrand
  int nearest_index = tree_extending_from->getNearestVertex(q_sample);

  // Couldn't find new point to extend towards goal
  if (nearest_index == -1) {
    ret_flag = TRAPPED;
    return;
  }
  
  // Extend tree towards nearest state in other tree
  State q_nearest = tree_extending_from->getState(nearest_index);
  std::pair<State,bool> new_state = this->getNewState(q_nearest,q_sample,eps_);
  State q_new = new_state.first;

  // Check for collision using twenty intermediate states (can be increased if cutting through obstacles)
  if(checkCollision(q_nearest, q_new, 20)) {
    ret_flag = new_state.second ? REACHED : ADVANCED;
    new_index = tree_extending_from->addVertex(q_new);
    tree_extending_from->addEdge(nearest_index, new_index);
    return;
  }

  ret_flag = TRAPPED;
  new_index = -1;
  return;
}

void PlannerRRTConnect::connect(State q_new, int& ret_flag, int& new_index) {
  while(true) {
    this->extend(q_new, other_tree_, ret_flag, new_index);
    if (ret_flag != ADVANCED) break;
  }
}

State PlannerRRTConnect::random_valid_sample() {
  State rand_sample(dof_);

  while(true) {
    for (int i = 0; i < dof_; ++i) {
      // Wrap from -2pi to 2pi (allows for one wrap)
      rand_sample[i] = GetRand(0,2*PI);//((double) rand() / (RAND_MAX)) * 4 * PI - 2*PI;
    }

    if (IsValidArmConfiguration(rand_sample.data(),dof_,map_,x_size_,y_size_)) {
      return rand_sample;
    }
  }
}

State PlannerRRTConnect::get_intermediate_state(State q1, State q2, double r) {
  State q_interm(q1.size());

  for (int i = 0; i < q1.size(); ++i) {
    q_interm[i] = q1[i] + r*(q2[i] - q1[i]);
  }
  return q_interm;
}

std::pair<State,bool> PlannerRRTConnect::getNewState(State q1, State q2, double eps) {
  double dist = StateDistance(q1,q2);
  if (dist < eps) return std::make_pair(q2,true);
  
  return std::make_pair(this->getIntermediateState(q1,q2,eps/dist),false);
}