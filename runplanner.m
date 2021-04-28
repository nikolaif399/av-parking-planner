% Construct vehicle cost map
mapLayers.StationaryObstacles = imread('stationary.bmp');
mapLayers.RoadMarkings        = imread('road_markings.bmp');
mapLayers.ParkedCars          = imread('parked_cars.bmp');

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

% Load vehicle dimensions
vehicleDims = [4,2]; % length, width

% Decompose map into occupancy map
cellsize = 0.5; % meters
combinedMapBin = boolean(im2single(combinedMap));

% Call planner here
startState = [12.5;12.5;0];
goalState = [37.5;37.5;0];
goalState = [36;45;pi/2];

tic
[planStates,planLength] = planner(startState,goalState,vehicleDims,combinedMapBin',cellsize);
toc

costmap = vehicleCostmap(combinedMap, 'CellSize', cellsize);
animateTrajectory(costmap,planStates,vehicleDims);