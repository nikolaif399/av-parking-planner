# Autonomous Vehicle Parking Planner

### Sample animation using parking lot environment
![Parking Lot Environment](environment_video.gif)


### Build and Run instructions
(from matlab command terminal)

For main planner script:
```bash
cd av-parking-planner
mex planner.cpp code/*.cpp -Icode/*.h
runplanner
```

For collision detection debug script
```bash
cd av-parking-planner
mex collision_debug.cpp code/*.cpp -Icode/*.h
test_collision_detector
```

Currently the collision detector just computes the indices in the occupancy grid that need checked

