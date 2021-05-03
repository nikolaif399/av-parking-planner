#include "rrt_star.h" 

#define REACHED 0
#define ADVANCED 1
#define TRAPPED 2

PlannerRRTStar::PlannerRRTStar(int dof, double *map, int x_size, int y_size, int k, double eps, double goal_prob) : 
PlannerRRT(dof, map, x_size, y_size, k, eps, goal_prob){
  return_on_success_ = false;
}

void PlannerRRTStar::extend(State q_sample, bool connect2goal, int& ret_flag, int& new_index) {
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
    if (connect2goal && !new_state.second) {
      goal_conn_failed_.insert(nearest_index);
    }
    ret_flag = new_state.second ? REACHED : ADVANCED;

    this->rewire(nearest_index, new_index);

    return;
  }

  if (connect2goal) {
    goal_conn_failed_.insert(nearest_index);
  }

  ret_flag = TRAPPED;
  new_index = -1;
}

void PlannerRRTStar::rewire(int nearest_index, int new_index) {

  State q_nearest = tree_start_->getState(nearest_index);
  State q_new = tree_start_->getState(new_index);
  std::vector<int> neighbors = tree_start_->getNeighborVertices(q_new, 5*eps_);

  State q_min = q_nearest;
  int min_index = nearest_index;
  double nearest_cost = tree_start_->getCost(nearest_index) + StateDistance(q_new,q_nearest);
  double min_cost = nearest_cost;

  // Find best edge to new index
  for (int i = 0; i < neighbors.size(); ++i) {
    State q_near = tree_start_->getState(neighbors.at(i));
    if (checkCollision(q_near, q_new, 20)) {
      double cprime = tree_start_->getCost(neighbors.at(i)) + 
                      StateDistance(q_near,q_new);
      if (cprime < min_cost) {
        q_min = q_near;
        min_index = neighbors.at(i);
        min_cost = cprime;
      }
    }
  }
  tree_start_->addEdge(min_index,new_index);

  // Rewire edges close to new state
  for (int i = 0; i < neighbors.size(); ++i) {
    if (i == nearest_index) continue;

    State q_near = tree_start_->getState(neighbors.at(i));
    if (!checkCollision(q_new,q_near,20)) continue;
     // we have a collision free path from our new node to this state

    double cost_near = tree_start_->getCost(neighbors.at(i)); // current cost to this neighbor
    double cost_new = tree_start_->getCost(new_index); // cost to the newly added vertex
    double transition_cost = StateDistance(q_new,q_near); // cost of transition t neighbor

    if (cost_near <= cost_new + transition_cost) continue;

    // Going through our new node is better than the original, rewire
    tree_start_->addEdge(new_index,neighbors.at(i));
    //printf("Rewiring cost change: %f -> %f (should go to %f)\n", cost_near, tree_start_->getCost(neighbors.at(i)),cost_new + transition_cost);
  }

}

