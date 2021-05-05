
#include "tree.h"

Tree::Tree(int num_dofs) {
  num_dofs_ = num_dofs;
}

void Tree::debugPrint() {
  printf("Num Vertices: %d\n", num_vertices_);
  printf("Num Edges: %d\n", num_edges_);
}

int Tree::addVertex(State q) {
  int vertex_index = next_vertex_index_;
  states_[vertex_index] = q;
  next_vertex_index_++;
  num_vertices_++;
  return vertex_index;
}
void Tree::addEdge(int ind1, int ind2) {
  parents_[ind2] = ind1;
  num_edges_++;
}

State Tree::getState(int index){
  if (states_.find(index) == states_.end()) {
    printf("Tree::getState, Index %d not in tree\n", index);
  }
  return states_[index];
}

int Tree::getNearestVertex(State q) {
  int nearest_index = -1;
  double nearest_dist = std::numeric_limits<double>::max();
  for (auto const &entry : states_) {
    double dist = StateDistance(q,entry.second);
    if (dist < nearest_dist) {
      nearest_index = entry.first;
      nearest_dist = dist;
    }
  }
  return nearest_index;
}

std::vector<int> Tree::getNeighborVertices(State q, double epsilon) {
  std::vector<int> neighbors;
  for (auto const &entry : states_) {
    double dist = StateDistance(q,entry.second);
    if (dist < epsilon) {
      neighbors.push_back(entry.first);
    }
  }
  return neighbors;
}
/*
void Tree::saveTree(int x_size, std::string filename) {
  printf("Saving tree to text file... ");

  std::ofstream treeFile;
  treeFile.open(filename);

  int child_index, parent_index;
  State child, parent;

  for (auto const &entry : states_) {
    child_index = entry.first;
    child = entry.second;

    if (parents_.find(child_index) != parents_.end()) {

      parent_index = parents_[child_index];
      parent = this->getState(parent_index);

      std::pair<double,double> child_pos = GetEndPoint(child, x_size, num_dofs_);
      std::pair<double,double> parent_pos = GetEndPoint(parent, x_size, num_dofs_);

      treeFile << child_pos.first << " " << child_pos.second << " " << child_index << "\n";
      treeFile << parent_pos.first << " " << parent_pos.second << " " << parent_index << "\n\n";
    }
  }

  treeFile.close();
  printf("done.\n");
}
*/

std::vector<State> Tree::getPath(int start_index, int goal_index) {
  //printf("Backtracking path through tree... ");

  // Count how many elements there are in the proposed path
  int cur_ind = goal_index;
  std::vector<State> plan_states;
  while (cur_ind != start_index) {
    plan_states.push_back(this->getState(cur_ind));
    cur_ind = parents_[cur_ind];
  }
  plan_states.push_back(this->getState(start_index));
  std::reverse(plan_states.begin(),plan_states.end());

  //printf("done.\n");

  return plan_states;
}