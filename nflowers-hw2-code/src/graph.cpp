#include "graph.h"

typedef std::pair<int, double> Cell; // (index, priority)

struct cellCompLow
{
  bool operator()(Cell const &a, Cell b)
  {
    return a.second > b.second;
  }
};

struct cellCompHi
{
  bool operator()(Cell const &a, Cell b)
  {
    return a.second < b.second;
  }
};

Graph::Graph(int dof) {
  dof_ = dof;
}

int Graph::addState(State q) {
  int index = cur_index_;
  conn_[index] = {};
  data_[index] = q;
  component_[index] = std::unordered_set<int>();
  component_[index].insert(index); // node is automatically connected to itself
  cur_index_++;

  return index;
}

State Graph::getState(int index) {
  if (data_.find(index) == data_.end()) {
    printf("Graph::getState, Index %d not in graph\n", index);
  }

  return data_[index];
}

bool Graph::connected(int ind1, int ind2) {
  if (component_.find(ind1) == component_.end()) {
    printf("Graph::connected, Index %d not in graph\n", ind1);
  }
  if (component_.find(ind2) == component_.end()) {
    printf("Graph::connected, Index %d not in graph\n", ind2);
  }

  // These should always be equivalent!
  bool oneContainsTwo = component_[ind1].find(ind2) != component_[ind1].end();
  bool twoContainsOne = component_[ind2].find(ind1) != component_[ind2].end();

  return oneContainsTwo;
}

std::vector<int> Graph::getKNearestNeigbors(State q, int k) {

  std::priority_queue<Cell,std::vector<Cell>, cellCompHi> neighbor_queue;

  for (auto const &state : data_) { 
    double d = StateDistance(q, state.second);
    
    if (neighbor_queue.size() < k) {
      neighbor_queue.push(std::make_pair(state.first, d));
    }

    if (d < neighbor_queue.top().second) {
      neighbor_queue.pop();
      neighbor_queue.push(std::make_pair(state.first, d));
    }
  }

  std::vector<int> neighbors;
  
  while(!neighbor_queue.empty()) {
    neighbors.push_back(neighbor_queue.top().first);
    neighbor_queue.pop();
  }

  return neighbors;
}

int Graph::getComponentSize(int index) {
  if (component_.find(index) == component_.end()) {
    printf("Graph::getComponentSize, Index %d not in graph\n", index);
  }

  return component_[index].size();
}

void Graph::addEdge(int ind1, int ind2) {
  if (conn_.find(ind1) == conn_.end()) {
    printf("Graph::addEdge, Index %d not in graph\n", ind1);
  }
  if (conn_.find(ind2) == conn_.end()) {
    printf("Graph::addEdge, Index %d not in graph\n", ind2);
  }

  double d = StateDistance(this->getState(ind1),this->getState(ind2));
  conn_[ind1].push_back(std::make_pair(ind2,d));
  conn_[ind2].push_back(std::make_pair(ind1,d));

  // Merge the individual components 
  std::unordered_set<int> component_set;
  std::set_union(std::begin(component_[ind1]), std::end(component_[ind1]),
                 std::begin(component_[ind2]), std::end(component_[ind2]),
                 std::inserter(component_set, std::begin(component_set)));
  
  // Propogate edge back to rest of component
  for(const auto& elem: component_set) {
    component_[elem] = component_set;
  }
}

std::vector<int> Graph::getNeighbors(State q, double neighborhood) {
  std::vector<int> neighbors;

  for (auto const &state : data_) { 
    double d = StateDistance(q, state.second);
    if (d < neighborhood) {
      neighbors.push_back(state.first);
    }
  }
  return neighbors;
}

std::vector<State> Graph::search(int start_index, int goal_index) {
  printf("Starting graph search\n");

  // Dijkstra until goal index reached
  std::map<int, int> parents;
  std::priority_queue<Cell,std::vector<Cell>, cellCompLow> open_list;
  std::unordered_set<int> closed_list;

  open_list.push(std::make_pair(start_index,0));

  while(!open_list.empty()) {
    // Pop lowest g val node off our pq
    Cell cur_cell = open_list.top();
    open_list.pop();
    
    // Extract cell index and heuristic from priority queue pair
    int cur_cell_idx = cur_cell.first;
    double cur_cell_heuristic_val = cur_cell.second;

    // Add to closed list so we don't look at this node anymore
    closed_list.insert(cur_cell_idx);

    std::vector<std::pair<int,double>> connections = conn_[cur_cell_idx];

    for (int i = 0; i < connections.size(); ++i) {
      std::pair<int,double> neighbor = connections.at(i);
      
      int neighbor_index = neighbor.first;
      int neighbor_transition_cost = neighbor.second;

      bool in_closed_list = closed_list.find(neighbor_index) != closed_list.end();
      if (in_closed_list) {
        continue;
      }
      else{
        open_list.push(std::make_pair(neighbor_index, cur_cell_heuristic_val+neighbor_transition_cost));
      }

      parents[neighbor_index] = cur_cell_idx;

      if (neighbor_index == goal_index) {
        printf("Dijkstra found goal!\n");

        return this->getPath(start_index, goal_index, parents);
      }
    }
  }
}

std::vector<State> Graph::getPath(int start_index, int goal_index, std::map<int, int> parents) {
  std::vector<State> path;
  
  int cur_index = goal_index;
  while(true) {
    path.push_back(this->getState(cur_index));
    if (cur_index == start_index) break;
    cur_index = parents[cur_index];
  }

  std::reverse(path.begin(), path.end());
  return path;
}

void Graph::saveGraph(int x_size) {
  printf("Saving graph to text file... ");

  std::ofstream edgeFile;
  edgeFile.open("prm_edges.txt");

  for (auto node : data_) {
    int cur_index = node.first;
    std::pair<double,double> cur_state = GetEndPoint(node.second, x_size, dof_);
    if (conn_.find(cur_index) != conn_.end() && conn_[cur_index].size() > 0) {
      std::vector<std::pair<int,double>> connections = conn_[cur_index];
      for (int i = 0; i < connections.size(); ++i) {
        if (connections.at(i).first > cur_index) {
          
          std::pair<double,double> next_state = GetEndPoint(data_[connections.at(i).first], x_size, dof_);
          edgeFile << cur_state.first << " " << cur_state.second << " " << cur_index << "\n";
          edgeFile << next_state.first << " " << next_state.second << " " << connections.at(i).first << "\n\n";
        }
      }
    }
  }

  edgeFile.close();
  printf("done.\n");
}