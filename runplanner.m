clear all
close all
clc
pause(1)
mex planner.cpp code/*.cpp -Icode/*.h
pause(1)
clear all

% Construct vehicle cost map
mapLayers.StationaryObstacles = imread('env/stationary.bmp');
mapLayers.RoadMarkings        = imread('env/road_markings.bmp');
mapLayers.ParkedCars          = imread('env/parked_cars.bmp');

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

% Load vehicle dimensions
vehicleDims = [4,2]; % length, width

% Decompose map into occupancy map
cellsize = 0.5; % meters
combinedMapBin = boolean(im2single(combinedMap));

% Call planner here
startState = [16 8 0];
goalStates = [16 16 pi/2;
              ];

tic
[planStates,planLength] = planner(startState,goalStates,vehicleDims,combinedMapBin',cellsize);
toc

costmap = vehicleCostmap(combinedMap, 'CellSize', cellsize);
planStates = preprocess_data(planStates,1);
animateTrajectory(startState, goalStates, costmap,planStates,vehicleDims);