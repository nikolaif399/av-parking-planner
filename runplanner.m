% Load vehicle cost map
mapLayers = loadParkingLotMapLayers;
combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = boolean(im2single(combinedMap));

costmap = combineMapLayers(mapLayers);

% Load vehicle dimensions
vehicleDims = [4,2]; % length, width

% Call planner here
startState = [10;5;0;0;0];
goalState = [57.5;45;pi/2;0;0];
[planStates,planLength] = planner(startState,goalState,vehicleDims,combinedMap',costmap.CellSize);

animateTrajectory(costmap,planStates,vehicleDims);

% Utility Functions for Loading Parking Lot
function mapLayers = loadParkingLotMapLayers()
%loadParkingLotMapLayers
%   Load occupancy maps corresponding to 3 layers - obstacles, road
%   markings, and used spots.

mapLayers.StationaryObstacles = imread('stationary.bmp');
mapLayers.RoadMarkings        = imread('road_markings.bmp');
mapLayers.ParkedCars          = imread('parked_cars.bmp');
end

function costmap = combineMapLayers(mapLayers)
%combineMapLayers
%   Combine map layers struct into a single vehicleCostmap.

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

res = 0.5; % meters
costmap = vehicleCostmap(combinedMap, 'CellSize', res);
end