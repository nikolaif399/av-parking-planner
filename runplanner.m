% Load vehicle cost map
mapLayers = loadParkingLotMapLayers;
costmap = combineMapLayers(mapLayers);

% Load vehicle dimensions
vehicleDims = [4,2]; % length, width

% Call planner here
startState = [10;10;0.4;0];
goalState = [40;20;0.6;0];
[planStates,planLength] = planner(startState,goalState,vehicleDims);

fprintf("Plan returned of length %d\n",size(planStates,1));

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