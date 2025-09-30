function [overall_fleet_cost, parkingSpots] = runBiLevelSimulation(numTrucks, numTargets, numObstacles, lower_level_algo, upper_level_algo, save_dir)
% This function encapsulates the entire bi-level optimization simulation.

% =========================================================================
% SECTION 1: SYSTEM PARAMETERS AND SETUP
% =========================================================================
% (Parameters are now passed in or derived)
floorSizeX = 5; floorSizeY = 5; gridResolution = 0.1;
worldSizeX = 12; worldSizeY = 12;
gridSizeX_drone = floor(floorSizeX / gridResolution); gridSizeY_drone = floor(floorSizeY / gridResolution);
xOffset_drone = floorSizeX / 2; yOffset_drone = floorSizeY / 2;
gridSizeX_truck = floor(worldSizeX / gridResolution); gridSizeY_truck = floor(worldSizeY / gridResolution);
xOffset_truck = worldSizeX / 2; yOffset_truck = worldSizeY / 2;

numNodes = 10; numParkingSpots = numTrucks; 
truckSpeed = 36.0; truckDiagFlightEnabled = false; 
maxMissionTime = 1.5; 

numDrones = numTrucks;
droneSpeed_H_base = 25.0; droneSpeed_V_base = 10.0;
maxPackageWeight = 20.0; % kg
currentPackageWeight = rand() * maxPackageWeight; % Random weight for this run
payloadSpeedFactor = 0.4;
droneCruiseAltitude = 0.120; droneStayTime = 5 / 3600;
droneBatteryMin = 0.1; initialBattery = 1.0; 
droneDiagFlightEnabled = true;
droneSafetyRadius = 0.005;

speedReduction = (currentPackageWeight / maxPackageWeight) * payloadSpeedFactor;
droneSpeed_H = droneSpeed_H_base * (1 - speedReduction);
droneSpeed_V = droneSpeed_V_base * (1 - speedReduction);

cost = struct();
cost.truckPerKm = 1.50; cost.truckPerHour = 25.00;
cost.dronePerHour = 20.00; cost.obstaclePenalty = 1000.00;
cost.batteryPenalty = 0.10; cost.signalPenalty = 500.00;
cost.dischargePenalty = 2000.00; cost.maxDischargeRate = 0.5;

ga.populationSize = 50; ga.maxGenerations = 20; ga.mutationRate = 0.1;
boa.populationSize = 20; boa.maxGenerations = 20;
boa.fmin = 0; boa.fmax = 2; boa.loudness = 0.5; boa.pulseRate = 0.5;

R1 = 8; R2 = 10; k1 = 0.1; k2 = 100;

% =========================================================================
% SECTION 2: ENVIRONMENT AND NODE GENERATION
% =========================================================================
% (Code for environment generation remains here)
targetPositions = zeros(numTargets, 2);
for i = 1:numTargets, targetPositions(i, 1) = (rand() * floorSizeX) + (xOffset_truck - xOffset_drone); targetPositions(i, 2) = (rand() * floorSizeY) + (yOffset_truck - yOffset_drone); end
targetPositions_local = targetPositions - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone];
target_indices_local = floor(targetPositions_local / gridResolution) + 1;
gridMap_drone = false(gridSizeY_drone, gridSizeX_drone);
obstacles = struct('x', {}, 'y', {}, 'w', {}, 'h', {}, 'z', {});
safetyRadius_cells = ceil(droneSafetyRadius / gridResolution);
for i = 1:numObstacles
    isPlacementValid = false;
    while ~isPlacementValid
        obsW_km = 0.1 + rand()*0.3; obsH_km = 0.1 + rand()*0.3;
        obsW = floor(obsW_km / gridResolution); obsH = floor(obsH_km / gridResolution);
        obsX = randi(gridSizeX_drone - obsW); obsY = randi(gridSizeY_drone - obsH);
        isClearOfTargets = true;
        expanded_x_min = obsX - safetyRadius_cells; expanded_x_max = obsX + obsW - 1 + safetyRadius_cells;
        expanded_y_min = obsY - safetyRadius_cells; expanded_y_max = obsY + obsH - 1 + safetyRadius_cells;
        for j = 1:numTargets
            tx = target_indices_local(j, 1); ty = target_indices_local(j, 2);
            if (tx >= expanded_x_min && tx <= expanded_x_max && ty >= expanded_y_min && ty <= expanded_y_max)
                isClearOfTargets = false; break;
            end
        end
        if isClearOfTargets
            isPlacementValid = true;
            obsZ = (0.024 + rand() * 0.084);
            gridMap_drone(obsY:obsY+obsH-1, obsX:obsX+obsW-1) = true;
            obstacles(end+1) = struct('x', obsX, 'y', obsY, 'w', obsW, 'h', obsH, 'z', obsZ);
        end
    end
end
inflatedMap_drone = gridMap_drone;
[obs_y, obs_x] = find(gridMap_drone);
for i = 1:length(obs_x)
    for dy = -safetyRadius_cells:safetyRadius_cells
        for dx = -safetyRadius_cells:safetyRadius_cells
            if sqrt(dx^2 + dy^2) <= safetyRadius_cells
                ny = obs_y(i) + dy; nx = obs_x(i) + dx;
                if ny > 0 && ny <= gridSizeY_drone && nx > 0 && nx <= gridSizeX_drone, inflatedMap_drone(ny, nx) = true; end
            end
        end
    end
end
truckGridMap = false(gridSizeY_truck, gridSizeX_truck);
noGo_x_start = (xOffset_truck - floorSizeX/2); noGo_y_start = (yOffset_truck - floorSizeY/2);
noGo_x_start_idx = floor(noGo_x_start / gridResolution) + 1;
noGo_x_end_idx = floor((xOffset_truck + floorSizeX/2) / gridResolution) + 1;
noGo_y_start_idx = floor(noGo_y_start / gridResolution) + 1;
noGo_y_end_idx = floor((yOffset_truck + floorSizeY/2) / gridResolution) + 1;
truckGridMap(noGo_y_start_idx:noGo_y_end_idx, noGo_x_start_idx:noGo_x_end_idx) = true;
centralPoint = [xOffset_truck, yOffset_truck];
nodeRadius = 4.0; % km
truckNetworkNodes = zeros(numNodes, 2);
for i = 1:numNodes, angle = 2 * pi * i / numNodes; truckNetworkNodes(i, :) = centralPoint + nodeRadius * [cos(angle), sin(angle)]; end
depotLocation = [xOffset_truck, yOffset_truck + 5.0];

% =========================================================================
% SECTION 3: BI-LEVEL OPTIMIZATION (This will be a function call)
% =========================================================================
    % This section will be replaced by a call to the appropriate
    % lower-level and upper-level algorithm functions based on the
    % 'lower_level_algo' and 'upper_level_algo' inputs.
    % For now, we use the existing GA+BOA as a placeholder.
    
    if strcmp(lower_level_algo, 'GA') && strcmp(upper_level_algo, 'BOA')
        [overall_fleet_cost, parkingSpots, ga_hist, boa_hist, jtruck_comp, jdrone_comp, truck_paths, drone_paths, drone_params] = ...
            solve_with_GA_BOA(numTrucks, numNodes, ga, boa, cost, maxMissionTime, truckGridMap, truckDiagFlightEnabled, ...
            worldSizeX, worldSizeY, truckNetworkNodes, targetPositions, inflatedMap_drone, ...
            droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, ...
            droneBatteryMin, initialBattery, xOffset_truck, yOffset_truck, xOffset_drone, yOffset_drone, ...
            R1, R2, k1, k2, floorSizeX, floorSizeY);
    % Placeholder for other algorithms
    elseif strcmp(lower_level_algo, 'ALNS') && strcmp(upper_level_algo, 'PSO')
        % [overall_fleet_cost, parkingSpots, ...] = solve_with_ALNS_PSO(...);
        error('ALNS+PSO not yet implemented.');
    elseif strcmp(lower_level_algo, 'ACO') && strcmp(upper_level_algo, 'CMAES')
        % [overall_fleet_cost, parkingSpots, ...] = solve_with_ACO_CMAES(...);
        error('ACO+CMAES not yet implemented.');
    else
        error('Unknown algorithm combination specified.');
    end

% =========================================================================
% SECTION 4: PLOTTING & SAVING
% =========================================================================
% (All plotting and saving functions are called here, using 'save_dir')
% This part is omitted for brevity but would contain the code from your
% original script's sections 5 through 9, modified to save files to the 'save_dir'.

end