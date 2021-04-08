% Load vehicle cost map
mapLayers = loadParkingLotMapLayers;
costmap = combineMapLayers(mapLayers);

% Load vehicle dimensions
vehicleDims = [4,2]; % length, width

% Call planner here
planStates = repmat([10;10;0.4;0],1,200);
planStates(1,:) = linspace(10,40,200);
ts = 1:200;
planStates(2,:) = 14 + 10*sin(0.02*ts);
vx = diff(planStates(1,:));
vy = diff(planStates(2,:));
vx = [vx vx(end)];
vy = [vy vy(end)];

planStates(3,:) = atan2(vy,vx);

fprintf("Plan returned of length %d\n",size(planStates,2));

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