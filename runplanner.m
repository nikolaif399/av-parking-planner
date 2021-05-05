% Construct vehicle cost map
mapLayers.StationaryObstacles = imread('env/stationary.bmp');
mapLayers.RoadMarkings        = imread('env/road_markings.bmp');
mapLayers.ParkedCars          = imread('env/parked_cars.bmp');

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

% Load vehicle dimensions
vehicleDims = [4,2.5]; % length, width

% Decompose map into occupancy map
cellsize = 0.5; % meters
combinedMapBin = boolean(im2single(combinedMap));

% Call planner here

possibleStartStates = [12.5 12.5 0;
                       65 37.5 0;
                       10 37.5 pi/3;
                       60 12.5 2*pi/3;
                       67.5 10 pi/2;
                       2.5 25 0];
                   
startState = possibleStartStates(randperm(size(possibleStartStates,1),1),:);

goalStates = [14 45 pi/2;
              14 5 -pi/2
              36 45 pi/2;
              45 5 -pi/2];

tic
[planStates,planLength] = planner(startState,goalStates,vehicleDims,combinedMapBin',cellsize);
toc

costmap = vehicleCostmap(combinedMap, 'CellSize', cellsize);
animateTrajectory(startState, goalStates, costmap,planStates,vehicleDims);