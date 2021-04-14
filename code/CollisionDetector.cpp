#include "CollisionDetector.h"
#include "mex.h"

CollisionDetector::CollisionDetector(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size) {
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
  x_size_ = x_size;
  y_size_ = y_size;

  occupancy_grid_ = new bool[x_size_ * y_size_];
  memcpy(occupancy_grid_, occupancy_grid, x_size_ * y_size_ * sizeof(bool));
  cell_size_ = cell_size;
}

CollisionDetector::~CollisionDetector() {
  delete[] occupancy_grid_;
}

void CollisionDetector::pointToGridIndices(double x, double y, int &x_ind, int &y_ind) {
  x_ind = floor(x/cell_size_);
  y_ind = floor(y/cell_size_);
}

//http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
std::vector<std::pair<int,int>> CollisionDetector::raster_line(double x0, double y0, double x1, double y1){
  std::vector<std::pair<int,int>> coords;

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
  double x = state.at(0);
  double y = state.at(1);
  double theta = state.at(2);

  // Find indices of four corners
  double x1 = x + vehicle_width_/2*sin(theta);
  double y1 = y - vehicle_width_/2*cos(theta);

  double x2 = x - vehicle_width_/2*sin(theta);
  double y2 = y + vehicle_width_/2*cos(theta);

  double x3 = x2 + vehicle_length_*cos(theta);
  double y3 = y2 + vehicle_length_*sin(theta);

  double x4 = x1 + vehicle_length_*cos(theta);
  double y4 = y1 + vehicle_length_*sin(theta);

  std::vector<std::pair<int,int>> l12_coords = this->raster_line(x1,y1,x2,y2);
  printf("L12 Found %d coords between (%f,%f) and (%f,%f).\n", l12_coords.size(),x1,y1,x2,y2);

  std::vector<std::pair<int,int>> l23_coords = this->raster_line(x2,y2,x3,y3);
  printf("L23 Found %d coords between (%f,%f) and (%f,%f).\n", l23_coords.size(),x2,y2,x3,y3);

  std::vector<std::pair<int,int>> l34_coords = this->raster_line(x3,y3,x4,y4);
  printf("L34 Found %d coords between (%f,%f) and (%f,%f).\n", l34_coords.size(),x3,y3,x4,y4);

  std::vector<std::pair<int,int>> l41_coords = this->raster_line(x4,y4,x1,y1);
  printf("L41 Found %d coords between (%f,%f) and (%f,%f).\n", l41_coords.size(),x4,y4,x1,y1);

  x = x/cell_size_;
  y = y/cell_size_;
  int index_test = GETMAPINDEX(x,y,x_size_,y_size_);

  return false;
}