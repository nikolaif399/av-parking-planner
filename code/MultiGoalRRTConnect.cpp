#include "MultiGoalRRTConnect.h"
#include <iostream>

MultiGoalRRTConnect::MultiGoalRRTConnect(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size, int k, double eps) {
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
  x_size_ = x_size;
  y_size_ = y_size;
  cell_size_ = cell_size;

  k_ = k;
  eps_ = eps;

  state_lo_ = {0,0,-M_PI};
  state_hi_ = {x_size*cell_size,y_size*cell_size,M_PI};

  tree_start_ = std::make_shared<Tree>(state_lo_.size());
  tree_goal_ = std::make_shared<Tree>(state_lo_.size());

  cur_tree_ = tree_start_;
  cur_equal_start_ = true;
  other_tree_ = tree_goal_;

  collision_detector_ = std::make_shared<CollisionDetector>(vehicle_length, vehicle_width, x_size, y_size, occupancy_grid, cell_size);
}

// Simple plan just to test mex bindings
std::vector<State> MultiGoalRRTConnect::planSimple(State start_state, State goal_state) {

	// Arbitrary plan to test mex bindings
  int plan_length = 200;
  std::vector<State> plan(plan_length);

  // Just interpolate from start to goal
  for (int i = 0; i < plan.size(); ++i) {
    plan.at(i) = GetIntermediateState(start_state,goal_state,(double)i/(plan_length-1));
    bool collision = collision_detector_->checkCollision(plan.at(i));
    //printf("Collision at index %d : %d\n", i, collision);
    plan.at(i).back() = collision ? 1 : 0;
  }

  return plan;
}

// Implement main plan here
std::vector<State> MultiGoalRRTConnect::plan(State start_state, State goal_state) {
  
  // Add start state to start tree
  int start_ind = tree_start_->addVertex(start_state);

  // Add goal state to goal tree
  int goal_ind = tree_goal_->addVertex(goal_state);
  
  // RRT-Connect iterations
  for (int i = 0; i < k_; ++i) { 
    printf("Iteration %d\n", i);

    // Draw a random valid sample from the state space
    State q_sample = this->random_valid_sample();

    //PrintState(q_sample);

    // Try to extend the current tree towards to the new sample
    int ret_flag,new_index_extend;
    this->extend(q_sample, cur_tree_, ret_flag, new_index_extend);

    // If not trapped, try to connect to other tree
    if (ret_flag != TRAPPED) {
      State q_new = cur_tree_->getState(new_index_extend);
      int new_index_connect;
      this->connect(q_new, ret_flag, new_index_connect);

      /*printf("Sampled state: \n");
      PrintState(q_sample);
      printf("New state: \n");
      PrintState(q_new);*/

      // If connect reached, we have a path from start to goal!
      // Compute the index of the midpoint state in both trees
      if (ret_flag == REACHED) {
        printf("Reached goal!!\n");
        printf("Connected through: \n");
        PrintState(q_new);

        // Found plan, exit
        std::vector<State> plan;

        return plan;
      }
    }

    // Exchange goal tree and start tree
    std::shared_ptr<Tree> temp = cur_tree_;
    cur_tree_ = other_tree_;
    other_tree_ = temp;
    cur_equal_start_ = !cur_equal_start_;
  }


  std::vector<State> plan;

  return plan;
}

void MultiGoalRRTConnect::extend(State q_sample, std::shared_ptr<Tree> tree_extending_from, int& ret_flag, int& new_index) {
  // Get nearest neighbor to qrand
  int nearest_index = tree_extending_from->getNearestVertex(q_sample);

  // Couldn't find new point to extend towards goal
  if (nearest_index == -1) {
    ret_flag = TRAPPED;
    return;
  }
  
  // Extend tree towards nearest state in other tree
  State q_nearest = tree_extending_from->getState(nearest_index);
  std::pair<State,bool> new_state = this->get_new_state(q_nearest,q_sample,eps_);
  State q_new = new_state.first;

  /*printf("Q nearest: ");
  PrintState(q_nearest);

  printf("Q new: ");
  PrintState(q_new);*/
  

  // Check for collision using twenty intermediate states (can be increased if cutting through obstacles)
  if(!collision_detector_->checkCollisionLine(q_nearest, q_new, 20)) {
    //printf("Collision free!")
    ret_flag = new_state.second ? REACHED : ADVANCED;
    new_index = tree_extending_from->addVertex(q_new);
    tree_extending_from->addEdge(nearest_index, new_index);
    return;
  }
  
  ret_flag = TRAPPED;
  new_index = -1;
  return;
}

void MultiGoalRRTConnect::connect(State q_new, int& ret_flag, int& new_index) {
  while(true) {
    this->extend(q_new, other_tree_, ret_flag, new_index);
    if (ret_flag != ADVANCED) break;
  }
}

std::pair<State,bool> MultiGoalRRTConnect::get_new_state(State q1, State q2, double eps) {
  double dist = StateDistance(q1,q2);
  if (dist < eps) return std::make_pair(q2,true);
  
  return std::make_pair(GetIntermediateState(q1,q2,eps/dist),false);
}

State MultiGoalRRTConnect::random_valid_sample() {

  int num_in_state = state_lo_.size();

  State rand_sample(num_in_state);

  while(true) {

    for (int i = 0; i < num_in_state; ++i) {
      rand_sample[i] = GetRand(state_lo_.at(i),state_hi_.at(i));
    }

    if (!collision_detector_->checkCollision(rand_sample)) {

      return rand_sample;
    }
  }
}

std::vector<State> MultiGoalRRTConnect::interpolatePath(std::vector<State> plan_states) {
  int N = 25;

  std::vector<State> new_path;
  int num_points;

  for (int i = 0; i < plan_states.size() - 1; ++i) {
    double dist = StateDistance(plan_states[i], plan_states[i+1]);
    num_points = MAX(dist * N,2);
    for (int j = 0 ; j < num_points; ++j) {
      double r = double(j)/double(num_points);
      State interp_state = GetIntermediateState(plan_states[i], plan_states[i+1], r);
      new_path.push_back(interp_state);
    }
  }

  return new_path;
}

std::vector<State> MultiGoalRRTConnect::shortcutPath(std::vector<State> plan_states) {
  bool converged = true;

  int i = 0;
  std::vector<State> new_path;
  new_path.push_back(plan_states.front());

  while(i < 20) {
    State active = new_path.back();

    for (int comp_ind = plan_states.size() - 1; comp_ind >= 0; comp_ind--) {
      State comp = plan_states.at(comp_ind);
      if (collision_detector_->checkCollisionLine(active,comp,500)) {
        // Shorter collision free path found!
        new_path.push_back(comp);
        if (comp_ind == plan_states.size() - 1) {
          return new_path;
        }
        break;
      }
    }
    i++;
  }
  
  return new_path;
}