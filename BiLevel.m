% =========================================================================
% BI-LEVEL OPTIMIZATION FOR TRUCK-DRONE LAST-MILE DELIVERY
%
% This script implements a bi-level optimization model for a last-mile
% delivery scenario.
%
% Lower-Level: Truck Routing with a Genetic Algorithm (GA) to minimize the
%              JTruck cost function.
% Upper-Level: Drone Path Planning with A* Search to find the shortest,
%              safest path, optimized by a Bat Algorithm (BOA).
%
% The two levels are linked, as the optimal flight time of the drone (from
% the Upper-Level) is a key variable in the truck's cost function.
% =========================================================================
clear; close all; clc;
% =========================================================================
% SECTION 1: SYSTEM PARAMETERS AND SETUP
% =========================================================================
% General Simulation Parameters
floorSizeX = 5;
floorSizeY = 5; 
gridResolution = 0.1;

% Drone-specific map dimensions
gridSizeX_drone = floorSizeX / gridResolution;
gridSizeY_drone = floorSizeY / gridResolution;
xOffset_drone = floorSizeX / 2;
yOffset_drone = floorSizeY / 2;

% Truck-specific map dimensions
worldSizeX = 12;
worldSizeY = 12;
gridSizeX_truck = worldSizeX / gridResolution;
gridSizeY_truck = worldSizeY / gridResolution;
xOffset_truck = worldSizeX / 2;
yOffset_truck = worldSizeY / 2;

% Tunable Target Parameters
numTargets = 5; % TUNABLE: Number of customers

% Truck Parameters (Lower-Level)
numTrucks = numTargets;
numNodes = 10; 
numParkingSpots = numTrucks; 
truckSpeed = 10; % m/s

% Genetic Algorithm Parameters (for Truck Routing)
ga.populationSize = 50;
ga.maxGenerations = 50;
ga.mutationRate = 0.1;

% Drone Parameters (Upper-Level)
numDrones = numTrucks;
droneSpeed_H = 5; % Horizontal speed (m/s)
droneSpeed_V = 2; % Vertical speed (m/s)
droneCruiseAltitude = 10; % meters
droneStayTime = 5; % seconds at target
droneBatteryMin = 0.1; 
initialBattery = 1.0; 
diagFlightEnabled = true; % TUNABLE: A* diagonal flight for drone
droneSafetyRadius = 0.3; % TUNABLE: Safety distance from obstacles (meters)

% A* Debug Flag
DEBUG_ASTAR = false; % Set to false to hide A* pathfinding details for speed

% Bat Optimization Algorithm Parameters (for Drone Flight)
boa.populationSize = 20;
boa.maxGenerations = 10;
boa.fmin = 0; boa.fmax = 2;
boa.loudness = 0.5;
boa.pulseRate = 0.5;

% Cost Function Weights (Tunable)
w1 = 0.5;  % Path Length
w2 = 0.1;  % Flight Time
w3 = 100;  % Obstacle Penalty (used in simulation, not A*)
w4 = 10; % Battery Usage Penalty
w5 = 5;    % Signal Coverage Penalty

% Signal Coverage Penalty Parameters
towerPosition = [xOffset_truck, yOffset_truck]; 
R1 = 2; % Safe range (no penalty) in meters
R2 = 4; % Max range (small penalty) in meters
k1 = 0.1; 
k2 = 100;
% =========================================================================
% SECTION 2: ENVIRONMENT AND NODE GENERATION
% =========================================================================
% --- Generate Randomized Target Positions ---
targetPositions = zeros(numTargets, 2);
for i = 1:numTargets
    targetPositions(i, 1) = (rand() * floorSizeX) + (xOffset_truck - xOffset_drone);
    targetPositions(i, 2) = (rand() * floorSizeY) + (yOffset_truck - yOffset_drone);
end
targetPositions_local = targetPositions - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone];
target_indices_local = floor(targetPositions_local / gridResolution) + 1;

% --- MAP 1: Drone Operating Area with Varied Obstacles and Target Safety Check ---
gridMap_drone = zeros(gridSizeY_drone, gridSizeX_drone);
obstacles = struct('x', {}, 'y', {}, 'w', {}, 'h', {}, 'z', {});
numObstacles = 25;
safetyRadius_cells = ceil(droneSafetyRadius / gridResolution);

for i = 1:numObstacles
    isPlacementValid = false;
    while ~isPlacementValid
        obsW = randi([2, 5]); obsH = randi([2, 5]);
        obsX = randi(gridSizeX_drone - obsW);
        obsY = randi(gridSizeY_drone - obsH);
        isClearOfTargets = true;
        expanded_x_min = obsX - safetyRadius_cells; expanded_x_max = obsX + obsW - 1 + safetyRadius_cells;
        expanded_y_min = obsY - safetyRadius_cells; expanded_y_max = obsY + obsH - 1 + safetyRadius_cells;
        for j = 1:numTargets
            tx = target_indices_local(j, 1); ty = target_indices_local(j, 2);
            if (tx >= expanded_x_min && tx <= expanded_x_max && ty >= expanded_y_min && ty <= expanded_y_max)
                isClearOfTargets = false;
                break;
            end
        end
        if isClearOfTargets
            isPlacementValid = true;
            obsZ = (rand() * 0.7 + 0.2) * droneCruiseAltitude;
            gridMap_drone(obsY:obsY+obsH-1, obsX:obsX+obsW-1) = 1;
            obstacles(end+1) = struct('x', obsX, 'y', obsY, 'w', obsW, 'h', obsH, 'z', obsZ);
        end
    end
end

% --- Inflate Drone Map for Safety Radius ---
inflatedMap_drone = gridMap_drone;
[obs_y, obs_x] = find(gridMap_drone == 1);
for i = 1:length(obs_x)
    for dy = -safetyRadius_cells:safetyRadius_cells
        for dx = -safetyRadius_cells:safetyRadius_cells
            if sqrt(dx^2 + dy^2) <= safetyRadius_cells
                ny = obs_y(i) + dy; nx = obs_x(i) + dx;
                if ny > 0 && ny <= gridSizeY_drone && nx > 0 && nx <= gridSizeX_drone
                    inflatedMap_drone(ny, nx) = 1;
                end
            end
        end
    end
end

% --- MAP 2: Truck Road Network ---
truckGridMap = zeros(gridSizeY_truck, gridSizeX_truck);
noGo_x_start = (xOffset_truck - floorSizeX/2); noGo_y_start = (yOffset_truck - floorSizeY/2);
noGo_x_start_idx = floor(noGo_x_start / gridResolution) + 1;
noGo_x_end_idx = floor((xOffset_truck + floorSizeX/2) / gridResolution) + 1;
noGo_y_start_idx = floor((yOffset_truck - floorSizeY/2) / gridResolution) + 1;
noGo_y_end_idx = floor((yOffset_truck + floorSizeY/2) / gridResolution) + 1;
truckGridMap(noGo_y_start_idx:noGo_y_end_idx, noGo_x_start_idx:noGo_x_end_idx) = 1;

% --- Node and Depot Generation ---
centralPoint = [xOffset_truck, yOffset_truck];
nodeRadius = 4;
truckNetworkNodes = zeros(numNodes, 2);
for i = 1:numNodes
    angle = 2 * pi * i / numNodes;
    truckNetworkNodes(i, :) = centralPoint + nodeRadius * [cos(angle), sin(angle)];
end
depotLocation = [xOffset_truck, yOffset_truck + 5];

% =========================================================================
% SECTION 3: LOWER-LEVEL OPTIMIZATION (TRUCK ROUTING - GA)
% =========================================================================
bestOverallCost = inf; bestOverallRoute = cell(numTrucks, 1); bestSelectedSpots = []; 
if isempty(gcp('nocreate')), parpool; end
for gen = 1:ga.maxGenerations
    if gen == 1, population = zeros(ga.populationSize, numNodes); for p = 1:ga.populationSize, population(p, :) = randperm(numNodes); end
    else
        [~, sortedIndices] = sort(sum(costMatrix, 1)); newPopulation = population(sortedIndices(1:ga.populationSize/2), :);
        for i = 1:ga.populationSize/2
            parent1 = newPopulation(randi(size(newPopulation, 1)), :); parent2 = newPopulation(randi(size(newPopulation, 1)), :); crossoverPoint = randi(numNodes-1);
            child = zeros(1, numNodes); child(1:crossoverPoint) = parent1(1:crossoverPoint); p2_elements = parent2(~ismember(parent2, child));
            if numNodes - crossoverPoint <= length(p2_elements), child(crossoverPoint+1:end) = p2_elements(1:numNodes-crossoverPoint);
            else, child(crossoverPoint+1:end) = p2_elements(1:min(length(p2_elements), numNodes-crossoverPoint)); end
            newPopulation(i+ga.populationSize/2, :) = child;
        end
        for i = ga.populationSize/2 + 1 : ga.populationSize
            if rand < ga.mutationRate, mutationPoints = randperm(numNodes, 2); temp = newPopulation(i, mutationPoints(1)); newPopulation(i, mutationPoints(1)) = newPopulation(i, mutationPoints(2)); newPopulation(i, mutationPoints(2)) = temp; end
        end
        population = newPopulation;
    end
    costMatrix = zeros(numTrucks, ga.populationSize);
    parfor p = 1:ga.populationSize
        currentRouteIndices = population(p, :); currentCostForIndividual = zeros(numTrucks, 1); selectedParkingNodeIndices = currentRouteIndices(1:numTrucks);
        for t = 1:numTrucks
            parkingNodeIndex = selectedParkingNodeIndices(t); startNode = truckNetworkNodes(parkingNodeIndex, :); truckTarget = targetPositions(mod(t-1, size(targetPositions, 1)) + 1, :);
            [~, truckTravelDistance] = AStarPath(truckGridMap, depotLocation, startNode, false, worldSizeX, worldSizeY, false);
            if isinf(truckTravelDistance), currentCostForIndividual(t) = 1e9; continue; end
            truckTravelTime = truckTravelDistance / truckSpeed;
            startNode_local = startNode - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone]; truckTarget_local = truckTarget - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone];
            [~, droneFlightCost] = runDroneBat(startNode_local, truckTarget_local, inflatedMap_drone, w1, w2, w3, w4, w5, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, diagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, boa, [xOffset_drone, yOffset_drone], R1, R2, k1, k2, floorSizeX, floorSizeY);
            droneTravelTime = droneFlightCost; truckWaitTime = max(0, droneTravelTime - truckTravelTime);
            alpha1 = 0.1; alpha2 = 0.1; alpha3 = 0.9; alpha4 = 0.1; JTruck = alpha1 * truckTravelDistance + alpha2 * truckWaitTime + alpha3 * droneTravelTime + alpha4 * truckTravelTime;
            currentCostForIndividual(t) = JTruck;
        end
        costMatrix(:, p) = currentCostForIndividual;
    end
    [minGenCost, bestIdx] = min(sum(costMatrix, 1));
    if minGenCost < bestOverallCost
        bestOverallCost = minGenCost; bestRouteIndices = population(bestIdx, :); bestSelectedSpots = truckNetworkNodes(bestRouteIndices(1:numTrucks), :);
        for t = 1:numTrucks, bestOverallRoute{t} = [depotLocation; bestSelectedSpots(t, :)]; end
    end
end
parkingSpots = bestSelectedSpots;
final_JTruck_costs = zeros(numTrucks, 1);
final_JDrone_costs = zeros(numTrucks, 1);
final_truck_paths = cell(numTrucks, 1);
final_drone_paths = cell(numTrucks, 1);
final_drone_params = cell(numTrucks, 1);
% =========================================================================
% SECTION 4: FINAL PATH & COST CALCULATION
% =========================================================================
for t = 1:numTrucks
    startPos_world = parkingSpots(t, :);
    targetPos_world = targetPositions(mod(t-1, size(targetPositions, 1)) + 1, :);
    startPos_local = startPos_world - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone];
    targetPos_local = targetPos_world - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone];
    [bestParams, droneFlightCost] = runDroneBat(startPos_local, targetPos_local, inflatedMap_drone, w1, w2, w3, w4, w5, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, diagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, boa, [xOffset_drone, yOffset_drone], R1, R2, k1, k2, floorSizeX, floorSizeY);
    final_JDrone_costs(t) = droneFlightCost;
    final_drone_params{t} = bestParams;
    [truckPath_meters, truckTravelDistance] = AStarPath(truckGridMap, depotLocation, startPos_world, false, worldSizeX, worldSizeY, false);
    final_truck_paths{t} = truckPath_meters;
    truckTravelTime = truckTravelDistance / truckSpeed;
    droneTravelTime = droneFlightCost;
    truckWaitTime = max(0, droneTravelTime - truckTravelTime);
    alpha1 = 0.1; alpha2 = 0.1; alpha3 = 0.9; alpha4 = 0.1;
    JTruck = alpha1 * truckTravelDistance + alpha2 * truckWaitTime + alpha3 * droneTravelTime + alpha4 * truckTravelTime;
    final_JTruck_costs(t) = JTruck;
    [pathXY_local, ~] = AStarPath(inflatedMap_drone, startPos_local, targetPos_local, diagFlightEnabled, floorSizeX, floorSizeY, false);
    [dronePath, ~, ~, ~, ~, ~] = simulateDroneFlight(startPos_local, targetPos_local, bestParams, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneStayTime, diagFlightEnabled, gridMap_drone, [xOffset_drone, yOffset_drone], R1, R2, k1, k2, pathXY_local);
    final_drone_paths{t} = dronePath;
end
fprintf('\n--- FINAL OPTIMIZED RESULTS ---\n');
for t = 1:numTrucks, fprintf('  Truck %d -> Target %d: JTruck=%.2f, JDrone=%.2f\n', t, mod(t-1, size(targetPositions, 1)) + 1, final_JTruck_costs(t), final_JDrone_costs(t)); end
fprintf('--------------------------------\n\n');
% =========================================================================
% SECTION 5: STATIC FINAL STATE VISUALIZATION
% =========================================================================
figure('Name', 'Static Final Solution', 'NumberTitle', 'off', 'Color', 'w', 'Position', [50, 50, 1200, 600]);
% --- SUBPLOT 1: Truck's World and Route ---
ax1_static = subplot(1, 2, 1);
hold on;
ax1_static.Color = [0.9 0.9 0.9];
rectangle('Position', [noGo_x_start, noGo_y_start, floorSizeX, floorSizeY], 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'k');
axis on; grid on;
title('Lower-Level: Final Truck Route');
xlabel('X (m)'); ylabel('Y (m)');
axis([0 worldSizeX 0 worldSizeY]);
truckColors = lines(numTrucks);
scatter(truckNetworkNodes(:, 1), truckNetworkNodes(:, 2), 50, 'b', 'filled', 'MarkerFaceAlpha', 0.8, 'DisplayName', 'Network Nodes');
scatter(parkingSpots(:, 1), parkingSpots(:, 2), 100, 'r', 'filled', 'Marker', 's', 'DisplayName', 'Parking Spots');
scatter(depotLocation(1), depotLocation(2), 200, 'k', 'filled', 'Marker', 'd', 'DisplayName', 'Depot');
for t = 1:numTrucks
    if ~isempty(final_truck_paths{t})
        plot(final_truck_paths{t}(:,1), final_truck_paths{t}(:,2), '--', 'LineWidth', 2, 'Color', truckColors(t,:), 'DisplayName', sprintf('Truck %d Route', t));
    end
end
scatter(targetPositions(:, 1), targetPositions(:, 2), 150, 'g', 'filled', 'Marker', 'p', 'DisplayName', 'Targets');
patch(NaN, NaN, [0.5 0.5 0.5], 'EdgeColor', 'k', 'DisplayName', 'No-Go Zone');
legend('Location', 'bestoutside');
hold off;

% --- SUBPLOT 2: Drone's World and Trajectory ---
ax2_static = subplot(1, 2, 2);
title('Upper-Level: Final Drone Trajectory');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; hold on;
% Adjusting axis limits dynamically based on nodeRadius
x_limits = [-nodeRadius, max(floorSizeX, nodeRadius*2) + nodeRadius/2]; % Added buffer
y_limits = [-nodeRadius, max(floorSizeY, nodeRadius*2) + nodeRadius/2]; % Added buffer
z_limits = [0, droneCruiseAltitude * 1.6]; % Slightly increased Z limit
axis([x_limits(1), x_limits(2), y_limits(1), y_limits(2), z_limits(1), z_limits(2)]);
view(3);
% Plot obstacles (only one legend entry for all)
for i = 1:length(obstacles)
    obs=obstacles(i); x=(obs.x-1)*gridResolution; y=(obs.y-1)*gridResolution; w=obs.w*gridResolution; h=obs.h*gridResolution; z=obs.z;
    vertices = [x,y,0; x+w,y,0; x+w,y+h,0; x,y+h,0; x,y,z; x+w,y,z; x+w,y+h,z; x,y+h,z];
    faces = [1,2,3,4; 5,6,7,8; 1,2,6,5; 2,3,7,6; 3,4,8,7; 4,1,5,8];
    if i == 1 % Only add DisplayName for the first obstacle to avoid duplicates
        patch('Vertices',vertices, 'Faces',faces, 'FaceColor',[0.3 0.3 0.3], 'EdgeColor','none', 'FaceAlpha',0.8, 'DisplayName', 'Obstacles');
    else
        patch('Vertices',vertices, 'Faces',faces, 'FaceColor',[0.3 0.3 0.3], 'EdgeColor','none', 'FaceAlpha',0.8, 'HandleVisibility', 'off');
    end
end
scatter3(xOffset_drone, yOffset_drone, droneCruiseAltitude, 200, 'm', 'filled', 'Marker', '^', 'DisplayName', 'Tower');
% Plot targets on the ground
scatter3(targetPositions_local(:,1), targetPositions_local(:,2), zeros(numTargets,1), 150, 'g', 'p', 'filled', 'DisplayName', 'Targets');
for t = 1:numTrucks
    if ~isempty(final_drone_paths{t})
        plot3(final_drone_paths{t}(:,1), final_drone_paths{t}(:,2), final_drone_paths{t}(:,3), 'Color', truckColors(t,:), 'LineWidth', 2, 'DisplayName', sprintf('Drone %d Path', t));
    end
end
legend('Location', 'best');
hold off;

% =========================================================================
% SECTION 6: SIMULATION ANIMATION
% =========================================================================
SAVE_ANIMATION = true; % TUNABLE: Set to true to save video and GIF

fprintf('Starting simulation animation...\n');
anim_fig = figure('Name', 'Live Simulation Animation', 'NumberTitle', 'off', 'Color', 'w', 'Position', [150, 150, 1200, 600]);
% --- Setup Video and GIF Writers ---
if SAVE_ANIMATION
    video_filename = 'simulation_video.mp4';
    v = VideoWriter(video_filename, 'MPEG-4');
    v.Quality = 95; v.FrameRate = 15;
    open(v);
    gif_filename = 'simulation_animation.gif';
end

% --- Setup Subplot 1: Truck's World ---
ax1_anim = subplot(1, 2, 1); hold on; ax1_anim.Color = [0.9 0.9 0.9];
rectangle('Position', [noGo_x_start, noGo_y_start, floorSizeX, floorSizeY], 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'k');
axis on; grid on; title('Lower-Level: Truck Movement'); xlabel('X (m)'); ylabel('Y (m)'); axis([0 worldSizeX 0 worldSizeY]);
scatter(truckNetworkNodes(:, 1), truckNetworkNodes(:, 2), 50, 'b', 'filled', 'MarkerFaceAlpha', 0.2);
scatter(parkingSpots(:, 1), parkingSpots(:, 2), 100, 'r', 'filled', 'Marker', 's');
scatter(depotLocation(1), depotLocation(2), 200, 'k', 'filled', 'Marker', 'd');
scatter(targetPositions(:,1), targetPositions(:,2), 150, 'g', 'p', 'filled');
for t = 1:numTrucks, if ~isempty(final_truck_paths{t}), plot(final_truck_paths{t}(:,1), final_truck_paths{t}(:,2), '--', 'Color', [truckColors(t,:), 0.3]); end; end

% --- Setup Subplot 2: Drone's World ---
ax2_anim = subplot(1, 2, 2); hold on; grid on; title('Upper-Level: Drone Movement'); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% Adjusting axis limits dynamically based on nodeRadius for animation too
axis([x_limits(1), x_limits(2), y_limits(1), y_limits(2), z_limits(1), z_limits(2)]);
view(3);
for i = 1:length(obstacles)
    obs=obstacles(i); x=(obs.x-1)*gridResolution; y=(obs.y-1)*gridResolution; w=obs.w*gridResolution; h=obs.h*gridResolution; z=obs.z;
    vertices = [x,y,0; x+w,y,0; x+w,y+h,0; x,y+h,0; x,y,z; x+w,y,z; x+w,y+h,z; x,y+h,z];
    faces = [1,2,3,4; 5,6,7,8; 1,2,6,5; 2,3,7,6; 3,4,8,7; 4,1,5,8];
    patch('Vertices',vertices, 'Faces',faces, 'FaceColor',[0.3 0.3 0.3], 'EdgeColor','none', 'FaceAlpha',0.8);
end
scatter3(targetPositions_local(:,1), targetPositions_local(:,2), zeros(numTargets,1), 150, 'g', 'p', 'filled');
for t = 1:numTrucks
    if ~isempty(final_drone_paths{t}), plot3(final_drone_paths{t}(:,1), final_drone_paths{t}(:,2), final_drone_paths{t}(:,3), '--', 'Color', [truckColors(t,:), 0.3]); end
end

% --- Prepare Time-Stamped Trajectories ---
truck_trajectories = cell(numTrucks, 1); drone_trajectories = cell(numTrucks, 1); max_time = 0;
for t = 1:numTrucks
    if ~isempty(final_truck_paths{t})
        path = final_truck_paths{t}; dist_segments = vecnorm(diff(path, 1, 1), 2, 2); cum_dist = [0; cumsum(dist_segments)];
        truck_time = cum_dist / truckSpeed; truck_trajectories{t} = {truck_time, path}; truck_arrival_time = truck_time(end);
        startPos_local = parkingSpots(t,:) - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone];
        targetPos_local = targetPositions(mod(t-1,size(targetPositions,1))+1,:) - [xOffset_truck-xOffset_drone, yOffset_truck-yOffset_drone];
        [pathXY_local, ~] = AStarPath(inflatedMap_drone, startPos_local, targetPos_local, diagFlightEnabled, floorSizeX, floorSizeY, false);
        [dronePath, droneTime] = simulateDroneFlight(startPos_local, targetPos_local, final_drone_params{t}, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneStayTime, diagFlightEnabled, gridMap_drone, [xOffset_drone,yOffset_drone], R1,R2,k1,k2, pathXY_local);
        droneTime = droneTime + truck_arrival_time; drone_trajectories{t} = {droneTime, dronePath};
        if ~isempty(droneTime) && droneTime(end) > max_time, max_time = droneTime(end); end
        if truck_arrival_time > max_time, max_time = truck_arrival_time; end
    end
end
% --- Initialize Animation Handles ---
h_truck_pos=gobjects(numTrucks,1); h_drone_pos=gobjects(numDrones,1); h_truck_trail=gobjects(numTrucks,1); h_drone_trail=gobjects(numDrones,1);
for t=1:numTrucks
    h_truck_pos(t)=plot(ax1_anim,NaN,NaN,'o','MarkerFaceColor',truckColors(t,:),'MarkerSize',10);
    h_truck_trail(t)=animatedline(ax1_anim,'Color',truckColors(t,:),'LineWidth',1.5);
    h_drone_pos(t)=plot3(ax2_anim,NaN,NaN,NaN,'p','MarkerFaceColor',truckColors(t,:),'MarkerSize',12);
    h_drone_trail(t)=animatedline(ax2_anim,'Color',truckColors(t,:),'LineWidth',1.5);
end
time_title = sgtitle(anim_fig, 'Simulation Time: 0.00s');
% --- Main Animation Loop ---
dt = 0.1;
for current_time = 0:dt:max_time
    for t = 1:numTrucks
        if ~isempty(truck_trajectories{t})
            truck_time_vec = truck_trajectories{t}{1}; truck_path_coords = truck_trajectories{t}{2};
            if current_time <= truck_time_vec(end), pos = interp1(truck_time_vec, truck_path_coords, current_time); set(h_truck_pos(t), 'XData', pos(1), 'YData', pos(2)); addpoints(h_truck_trail(t), pos(1), pos(2));
            else, set(h_truck_pos(t), 'XData', truck_path_coords(end,1), 'YData', truck_path_coords(end,2)); end
        end
        if ~isempty(drone_trajectories{t})
            drone_time_vec = drone_trajectories{t}{1}; drone_path_coords = drone_trajectories{t}{2};
            if current_time >= drone_time_vec(1) && current_time <= drone_time_vec(end), pos3d = interp1(drone_time_vec, drone_path_coords, current_time); set(h_drone_pos(t), 'XData', pos3d(1), 'YData', pos3d(2), 'ZData', pos3d(3)); addpoints(h_drone_trail(t), pos3d(1), pos3d(2), pos3d(3));
            elseif current_time > drone_time_vec(end), set(h_drone_pos(t), 'XData', drone_path_coords(end,1), 'YData', drone_path_coords(end,2), 'ZData', drone_path_coords(end,3)); end
        end
    end
    set(time_title, 'String', sprintf('Simulation Time: %.2fs', current_time));
    drawnow;
    if SAVE_ANIMATION
        frame = getframe(anim_fig);
        writeVideo(v, frame);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if current_time == 0, imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', dt);
        else, imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', dt); end
    end
end
set(time_title, 'String', sprintf('Simulation Complete! Total Time: %.2fs', max_time));
fprintf('Animation finished.\n');
if SAVE_ANIMATION, close(v); fprintf('Animation saved to %s and %s\n', video_filename, gif_filename); end

% =========================================================================
% SECTION 7: TRUCK NETWORK GRAPH VISUALIZATION
% =========================================================================
figure('Name', 'Truck Network Graph', 'NumberTitle', 'off');
hold on; ax_nodes = gca; ax_nodes.Color = [0.9 0.9 0.9]; 
rectangle('Position', [noGo_x_start, noGo_y_start, floorSizeX, floorSizeY], 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'k');
grid on;
for i = 1:numNodes, for j = i+1:numNodes, plot([truckNetworkNodes(i, 1), truckNetworkNodes(j, 1)], [truckNetworkNodes(i, 2), truckNetworkNodes(j, 2)], ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off'); end; end
scatter(depotLocation(1), depotLocation(2), 200, 'k', 'd', 'filled', 'DisplayName', 'Depot');
scatter(truckNetworkNodes(:, 1), truckNetworkNodes(:, 2), 70, 'b', 'o', 'filled', 'DisplayName', 'All Network Nodes');
scatter(parkingSpots(:, 1), parkingSpots(:, 2), 150, 'r', 's', 'filled', 'DisplayName', 'Selected Parking Spots');
for i = 1:numParkingSpots, text(parkingSpots(i, 1) + 0.1, parkingSpots(i, 2) + 0.1, sprintf('P%d', i), 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r'); end
patch(NaN, NaN, [0.5 0.5 0.5], 'EdgeColor', 'k', 'DisplayName', 'No-Go Zone');
title('Truck Network Nodes and Optimized Parking Spots'); xlabel('X (m)'); ylabel('Y (m)');
legend('show', 'Location', 'bestoutside'); axis equal; axis([0 worldSizeX 0 worldSizeY]); hold off;

% =========================================================================
% SECTION 8: SAVE RESULTS 
% =========================================================================
save('optimization_results.mat', 'final_truck_paths', 'parkingSpots', 'final_JTruck_costs', 'final_JDrone_costs');
fprintf('\nFinal results have been saved to optimization_results.mat\n');

% =========================================================================
% FUNCTION DEFINITIONS
% =========================================================================
function [bestParams, minCost] = runDroneBat(startPos, targetPos, gridMap, w1, w2, w3, w4, w5, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, diagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, boa, towerPosition, R1, R2, k1, k2, droneMapX, droneMapY), nVar = 3; varMin = [0.5, 0.5, 0.5]; varMax = [2.0, 2.0, 2.0]; bats = struct('Position', {}, 'Velocity', {}, 'Loudness', {}, 'PulseRate', {}, 'Cost', {}); [pathXY, ~] = AStarPath(gridMap, startPos, targetPos, diagFlightEnabled, droneMapX, droneMapY, false); if isempty(pathXY), minCost = 1e10; bestParams = [1, 1, 1]; return; end; for i = 1:boa.populationSize, bats(i).Position = varMin + (varMax - varMin).* rand(1, nVar); bats(i).Velocity = zeros(1, nVar); bats(i).Loudness = boa.loudness; bats(i).PulseRate = boa.pulseRate; bats(i).Cost = JDroneCost(bats(i).Position, startPos, targetPos, gridMap, w1, w2, w3, w4, w5, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, diagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); end; [minCost, bestBatIdx] = min([bats.Cost]); bestParams = bats(bestBatIdx).Position; for gen = 1:boa.maxGenerations, for i = 1:boa.populationSize, newF = boa.fmin + (boa.fmax - boa.fmin) * rand; bats(i).Velocity = bats(i).Velocity + (bats(i).Position - bestParams) * newF; newPos = bats(i).Position + bats(i).Velocity; newPos = max(newPos, varMin); newPos = min(newPos, varMax); if rand > bats(i).PulseRate, newPos = bestParams + 0.1 * randn(1, nVar); newPos = max(newPos, varMin); newPos = min(newPos, varMax); end; newCost = JDroneCost(newPos, startPos, targetPos, gridMap, w1, w2, w3, w4, w5, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, diagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); if newCost < bats(i).Cost && rand < bats(i).Loudness, bats(i).Position = newPos; bats(i).Cost = newCost; bats(i).Loudness = bats(i).Loudness * 0.9; bats(i).PulseRate = boa.pulseRate * (1 - exp(-gen/boa.maxGenerations)); end; end; [minCost, bestBatIdx] = min([bats.Cost]); bestParams = bats(bestBatIdx).Position; end; end
function cost = JDroneCost(params, startPos, targetPos, gridMap, w1, w2, w3, w4, w5, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, diagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY), [~, droneTimeVec, totalDistance, batteryConsumptionPerStep, obstaclePenaltySum, signalPenaltySum] = simulateDroneFlight(startPos, targetPos, params, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneStayTime, diagFlightEnabled, gridMap, towerPosition, R1, R2, k1, k2, pathXY); if isempty(droneTimeVec), totalTime = inf; else, totalTime = droneTimeVec(end); end; pathLengthIntegral = totalDistance; timePenaltyIntegral = totalTime; obstaclePenaltyIntegral = obstaclePenaltySum; remainingBattery = initialBattery - cumsum(batteryConsumptionPerStep); remainingBattery(remainingBattery < droneBatteryMin) = droneBatteryMin; batteryUsageIntegral = sum(1 ./ remainingBattery); signalCoverageIntegral = signalPenaltySum; cost = w1 * pathLengthIntegral + w2 * timePenaltyIntegral + w3 * obstaclePenaltyIntegral + w4 * batteryUsageIntegral + w5 * signalCoverageIntegral; end
function [path_meters, totalDistance] = AStarPath(gridMap, startPos, targetPos, diagEnabled, worldX, worldY, debug_flag), if nargin < 7, debug_flag = false; end; if debug_flag, fprintf('\n--- A* Debug Start ---\n'); fprintf('Attempting to find path from [%.2f, %.2f] to [%.2f, %.2f]\n', startPos(1), startPos(2), targetPos(1), targetPos(2)); end; [rows, cols] = size(gridMap); gridResolution_x = worldX / cols; gridResolution_y = worldY / rows; startIdx = floor(startPos ./ [gridResolution_x, gridResolution_y]) + 1; targetIdx = floor(targetPos ./ [gridResolution_x, gridResolution_y]) + 1; startIdx(1) = max(1, min(cols, startIdx(1))); startIdx(2) = max(1, min(rows, startIdx(2))); targetIdx(1) = max(1, min(cols, targetIdx(1))); targetIdx(2) = max(1, min(rows, targetIdx(2))); if debug_flag, fprintf('Start Index: [%d, %d], Target Index: [%d, %d]\n', startIdx(1), startIdx(2), targetIdx(1), targetIdx(2)); end; if gridMap(startIdx(2), startIdx(1)) == 1 || gridMap(targetIdx(2), targetIdx(1)) == 1, if debug_flag, fprintf('A* Error: Start or Target is inside an obstacle.\n'); end; path_meters = []; totalDistance = inf; return; end; openSet = priority_queue(); openSet.insert(startIdx, 0); gScore = inf(rows, cols); gScore(startIdx(2), startIdx(1)) = 0; cameFrom = zeros(rows, cols, 2); path = []; while ~openSet.isempty(), currentIdx = openSet.extract_min(); if isequal(currentIdx, targetIdx), path = []; current = currentIdx; while ~isequal(current, startIdx), path = [path; current]; current = squeeze(cameFrom(current(2), current(1), :))'; end; path = [path; startIdx]; path = flipud(path); break; end; neighbors = getNeighbors(currentIdx, diagEnabled, rows, cols); for i = 1:size(neighbors, 1), neighbor = neighbors(i, :); if neighbor(1) < 1 || neighbor(1) > cols || neighbor(2) < 1 || neighbor(2) > rows, continue; end; if gridMap(neighbor(2), neighbor(1)) == 1, continue; end; dist = norm((currentIdx - neighbor) .* [gridResolution_x, gridResolution_y]); tentative_gScore = gScore(currentIdx(2), currentIdx(1)) + dist; if tentative_gScore < gScore(neighbor(2), neighbor(1)), cameFrom(neighbor(2), neighbor(1), :) = currentIdx; gScore(neighbor(2), neighbor(1)) = tentative_gScore; hScore = norm((neighbor - targetIdx) .* [gridResolution_x, gridResolution_y]); fScore = tentative_gScore + hScore; if ~openSet.contains(neighbor), openSet.insert(neighbor, fScore); end; end; end; end; if isempty(path), if debug_flag, fprintf('A* Failure: No path found.\n'); end; path_meters = []; totalDistance = inf; else, if debug_flag, fprintf('A* Success: Path found with %d points.\n', size(path, 1)); end; path_meters = (path - 1) .* [gridResolution_x, gridResolution_y]; totalDistance = 0; for i = 1:size(path_meters, 1) - 1, totalDistance = totalDistance + norm(path_meters(i, :) - path_meters(i+1, :)); end; end; if debug_flag, fprintf('--- A* Debug End ---\n'); end; end
function neighbors = getNeighbors(node, diagEnabled, rows, cols), [x, y] = deal(node(1), node(2)); neighbors = []; if x > 1, neighbors = [neighbors; [x-1, y]]; end; if x < cols, neighbors = [neighbors; [x+1, y]]; end; if y > 1, neighbors = [neighbors; [x, y-1]]; end; if y < rows, neighbors = [neighbors; [x, y+1]]; end; if diagEnabled, if x > 1 && y > 1, neighbors = [neighbors; [x-1, y-1]]; end; if x < cols && y > 1, neighbors = [neighbors; [x+1, y-1]]; end; if x > 1 && y < rows, neighbors = [neighbors; [x-1, y+1]]; end; if x < cols && y < rows, neighbors = [neighbors; [x+1, y+1]]; end; end; end

function [dronePath, droneTime, totalDistance, batteryConsumptionPerStep, obstaclePenaltySum, signalPenaltySum] = simulateDroneFlight(startPos, targetPos, params, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneStayTime, diagEnabled, gridMap, towerPosition, R1, R2, k1, k2, pathXY), h_vel_scale = params(1); v_vel_scale_ascent = params(2); v_vel_scale_descent = params(3); gridResolution = 0.1; ascentSpeed = droneSpeed_V * v_vel_scale_ascent; ascentTime = droneCruiseAltitude / ascentSpeed; numStepsAscent = max(2, round(ascentTime / 0.1)); z_ascent = linspace(0, droneCruiseAltitude, numStepsAscent); path_ascent = [repmat(startPos(1), numStepsAscent, 1), repmat(startPos(2), numStepsAscent, 1), z_ascent']; time_ascent = linspace(0, ascentTime, numStepsAscent)'; horizSpeed = droneSpeed_H * h_vel_scale; path_horiz = [pathXY, repmat(droneCruiseAltitude, size(pathXY,1), 1)]; if size(pathXY, 1) > 1, dist_segments = vecnorm(diff(pathXY, 1, 1), 2, 2); time_segments = dist_segments / horizSpeed; time_horiz = [0; cumsum(time_segments)]; else, time_horiz = 0; end; descentSpeed = droneSpeed_V * v_vel_scale_descent; descentTime = droneCruiseAltitude / descentSpeed; numStepsDescent = max(2, round(descentTime / 0.1)); z_descent = linspace(droneCruiseAltitude, 0, numStepsDescent); path_descent = [repmat(targetPos(1), numStepsDescent, 1), repmat(targetPos(2), numStepsDescent, 1), z_descent']; time_descent = linspace(0, descentTime, numStepsDescent)'; dronePath = path_ascent; droneTime = time_ascent; if size(path_horiz, 1) > 1, dronePath = [dronePath; path_horiz(2:end, :)]; droneTime = [droneTime; droneTime(end) + time_horiz(2:end)]; end; dronePath = [dronePath; path_descent(2:end, :)]; droneTime = [droneTime; droneTime(end) + time_descent(2:end)]; dronePath = [dronePath; dronePath(end,:)]; droneTime = [droneTime; droneTime(end) + droneStayTime]; totalDistance = 0; if size(dronePath, 1) > 1, for i = 1:size(dronePath, 1) - 1, totalDistance = totalDistance + norm(dronePath(i, :) - dronePath(i+1, :)); end; end; batteryConsumptionPerStep = zeros(size(dronePath,1), 1); obstaclePenaltySum = 0; signalPenaltySum = 0; len_ascent_pts = size(path_ascent, 1); len_horiz_pts = size(path_horiz, 1); if isempty(pathXY), len_horiz_pts = 0; end; for i = 1:size(dronePath, 1), currentPos_2D = dronePath(i, 1:2); gridX_curr = floor(currentPos_2D(1)/gridResolution)+1; gridY_curr = floor(currentPos_2D(2)/gridResolution)+1; gridX_curr=max(1,min(size(gridMap,2),gridX_curr)); gridY_curr=max(1,min(size(gridMap,1),gridY_curr)); if gridMap(gridY_curr, gridX_curr) == 1, obstaclePenaltySum = obstaclePenaltySum + 1; end; distToTower = norm(currentPos_2D - towerPosition); if distToTower > R2, signalPenaltySum=signalPenaltySum+k2; elseif distToTower > R1, signalPenaltySum=signalPenaltySum+k1*(distToTower-R1)^2; end; len_horiz_segment = max(0, len_horiz_pts - 1); if i <= len_ascent_pts, batt_step = 0.01 + 0.005 * v_vel_scale_ascent; elseif i <= len_ascent_pts + len_horiz_segment, batt_step = 0.01 + 0.005 * h_vel_scale; else, batt_step = 0.01 + 0.005 * v_vel_scale_descent; end; batteryConsumptionPerStep(i) = batt_step; end; end