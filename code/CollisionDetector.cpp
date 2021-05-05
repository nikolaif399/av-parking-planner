#include "CollisionDetector.h"

typedef std::pair<int, int> Coord;

CollisionDetector::CollisionDetector(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size) {
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
  x_size_ = x_size;
  y_size_ = y_size;

  occupancy_grid_ = new bool[x_size_ * y_size_];
  memcpy(occupancy_grid_, occupancy_grid, x_size_ * y_size_ * sizeof(bool));
  //printf("Occupancy map size: x = %d, y = %d\n", x_size, y_size_);
  int occupied_cells = 0;
  for (int i = 0; i < x_size*y_size; ++i) {
    if (occupancy_grid_[i]) occupied_cells++;
  }
  //printf("Grid cells occupied: %d/%d (%.1f%%)\n", occupied_cells, x_size*y_size, static_cast<double>(occupied_cells)/x_size/y_size*100.0);
  //int index_test = GETMAPINDEX(79,90,x_size_,y_size_);
  //printf("39.5,45 colliding?: %d\n", occupancy_grid[index_test]);
  cell_size_ = cell_size;

  /*
  for (int x = 0; x < x_size; ++x) {
    for (int y = 0; y < y_size; ++y) {
      int index_test = GETMAPINDEX(x,y,x_size_,y_size_);
      if (occupancy_grid[index_test]) {
        printf("O");
      }
      else {
        printf(".");
      }
    }
    printf("\n");
  }
  */
}

CollisionDetector::~CollisionDetector() {
  delete[] occupancy_grid_;
}

void CollisionDetector::pointToGridIndices(double x, double y, int &x_ind, int &y_ind) {
  x_ind = floor(x/cell_size_);
  y_ind = floor(y/cell_size_);
}

//http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
std::vector<Coord> CollisionDetector::raster_line(double x0, double y0, double x1, double y1){
  std::vector<Coord> coords;

  double dx = fabs(x1 - x0);
  double dy = fabs(y1 - y0);

  int x = int(floor(x0));
  int y = int(floor(y0));

  int n = 1;
  int x_inc, y_inc;
  double error;

  if (dx == 0)
  {
    x_inc = 0;
    error = std::numeric_limits<double>::infinity();
  }
  else if (x1 > x0)
  {
    x_inc = 1;
    n += int(floor(x1)) - x;
    error = (floor(x0) + 1 - x0) * dy;
  }
  else
  {
    x_inc = -1;
    n += x - int(floor(x1));
    error = (x0 - floor(x0)) * dy;
  }

  if (dy == 0)
  {
    y_inc = 0;
    error -= std::numeric_limits<double>::infinity();
  }
  else if (y1 > y0)
  {
    y_inc = 1;
    n += int(floor(y1)) - y;
    error -= (floor(y0) + 1 - y0) * dx;
  }
  else
  {
    y_inc = -1;
    n += y - int(floor(y1));
    error -= (y0 - floor(y0)) * dx;
  }

  for (; n > 0; --n)
  {
    coords.push_back(std::make_pair(x, y));

    if (error > 0)
    {
      y += y_inc;
      error -= dx;
    }
    else
    {
      x += x_inc;
      error += dy;
    }
  }
  return coords;
}

// True means collision, false means free
bool CollisionDetector::checkCollision(State state) {
  double xcar = state.at(0);
  double ycar = state.at(1);
  double thetacar = state.at(2);

  double scar = sin(thetacar);
  double ccar = cos(thetacar);

  // Find indices of four corners in state space units
  double x1_a = xcar + vehicle_width_/2*scar;
  double y1_a = ycar - vehicle_width_/2*ccar;

  double x2_a = xcar - vehicle_width_/2*scar;
  double y2_a = ycar + vehicle_width_/2*ccar;

  double x1 = x1_a - 0.25*vehicle_length_*ccar;
  double y1 = y1_a - 0.25*vehicle_length_*scar;

  double x2 = x2_a - 0.25*vehicle_length_*ccar;
  double y2 = y2_a - 0.25*vehicle_length_*scar;

  double x3 = x2_a + vehicle_length_*ccar;
  double y3 = y2_a + vehicle_length_*scar;

  double x4 = x1_a + vehicle_length_*ccar;
  double y4 = y1_a + vehicle_length_*scar;

  // Convert all corners to occupancy grid measurements
  x1 /= cell_size_;
  y1 /= cell_size_;
  x2 /= cell_size_;
  y2 /= cell_size_;
  x3 /= cell_size_;
  y3 /= cell_size_;
  x4 /= cell_size_;
  y4 /= cell_size_;

  // Minimum and maximum values must lie at a corner
  int xmin = std::min({x1,x2,x3,x4});
  int xmax = std::max({x1,x2,x3,x4});
  int ymin = std::min({y1,y2,y3,y4});
  int ymax = std::max({y1,y2,y3,y4});

  // Check boundary conditions
  if (xmin < 0 || xmax >= x_size_) {
    return true;
  }

  if (ymin < 0 || ymax >= y_size_) {
    return true;
  }

  // Compute all indices along the border of the car
  std::vector<Coord> edge_indices;

  std::vector<Coord> l12_coords = this->raster_line(x1,y1,x2,y2);
  //printf("L12 Found %d coords between (%f,%f) and (%f,%f).\n", l12_coords.size(),x1,y1,x2,y2);
  edge_indices.insert(edge_indices.end(), l12_coords.begin(), l12_coords.end());

  std::vector<Coord> l23_coords = this->raster_line(x2,y2,x3,y3);
  //printf("L23 Found %d coords between (%f,%f) and (%f,%f).\n", l23_coords.size(),x2,y2,x3,y3);
  edge_indices.insert(edge_indices.end(), l23_coords.begin(), l23_coords.end());

  std::vector<Coord> l34_coords = this->raster_line(x3,y3,x4,y4);
  //printf("L34 Found %d coords between (%f,%f) and (%f,%f).\n", l34_coords.size(),x3,y3,x4,y4);
  edge_indices.insert(edge_indices.end(), l34_coords.begin(), l34_coords.end());

  std::vector<Coord> l41_coords = this->raster_line(x4,y4,x1,y1);
  //printf("L41 Found %d coords between (%f,%f) and (%f,%f).\n", l41_coords.size(),x4,y4,x1,y1);
  edge_indices.insert(edge_indices.end(), l41_coords.begin(), l41_coords.end());

  //printf("Xmin: %d\n", xmin);
  //printf("Xmax: %d\n", xmax);

  std::vector<int> ymins;
  ymins.assign(xmax - xmin + 1, y_size_ + 1);

  std::vector<int> ymaxs;
  ymaxs.assign(xmax - xmin + 1, -1);

  for (auto coord : edge_indices) {
    int xind = coord.first - xmin;
    if (xind < 0) {
      return true;
    }
    ymins.at(xind) = std::min(coord.second, ymins.at(xind));
    ymaxs.at(xind) = std::max(coord.second, ymaxs.at(xind));
  }

  
  for (int x = xmin; x <= xmax; ++x) {
    int xind = x - xmin;

    //printf("%d: %d -> %d\n", xind, ymins.at(xind), ymaxs.at(xind));
    for (int y = ymins.at(xind); y <= ymaxs.at(xind); ++y) {
      int index_test = GETMAPINDEX(x,y,x_size_,y_size_);
 
      if (occupancy_grid_[index_test]) {
        //printf("(%d,%d) colliding (grid space) -> (%f,%f) in state space\n",x,y,x*cell_size_,y*cell_size_);
        return true; // Collision
      }
    }
  }
  return false;
}

bool CollisionDetector::checkCollisionLine(State q1, State q2, int N = 100) {
  double dist = StateDistance(q1,q2); // normalized difference

  int numPoints = dist * N;

  for (int i = 0; i <= numPoints; ++i) {
    double r = static_cast<double>(i)/static_cast<double>(numPoints);
    State q = GetIntermediateState(q1,q2,r);
    if (this->checkCollision(q)) return true;
  }

  return false;
}