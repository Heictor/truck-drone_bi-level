function [overall_fleet_cost, parkingSpots] = solve_with_GA_BOA(numTrucks, numTargets, numObstacles, save_dir)
% =========================================================================
% BI-LEVEL OPTIMIZATION SOLVER: GENETIC ALGORITHM + BAT OPTIMIZATION
% =========================================================================

% --- Parameters and Environment Setup ---
% (This is the full setup code from your final working script)
floorSizeX = 5; floorSizeY = 5; gridResolution = 0.1;
worldSizeX = 12; worldSizeY = 12;
gridSizeX_drone = floor(floorSizeX/gridResolution); gridSizeY_drone = floor(floorSizeY/gridResolution);
xOffset_drone = floorSizeX/2; yOffset_drone = floorSizeY/2;
gridSizeX_truck = floor(worldSizeX/gridResolution); gridSizeY_truck = floor(worldSizeY/gridResolution);
xOffset_truck = worldSizeX/2; yOffset_truck = worldSizeY/2;
truckSpeed = 36.0; truckDiagFlightEnabled = false; maxMissionTime = 1.5;
droneSpeed_H_base = 25.0; droneSpeed_V_base = 10.0; maxPackageWeight = 20.0;
currentPackageWeight = rand() * maxPackageWeight;
payloadSpeedFactor = 0.4;
droneCruiseAltitude = 0.120; droneStayTime = 5/3600;
droneBatteryMin = 0.1; initialBattery = 1.0; droneDiagFlightEnabled = true;
droneSafetyRadius = 0.005;
cost = struct('truckPerKm',1.50,'truckPerHour',25.00,'dronePerHour',20.00,...
    'obstaclePenalty',1000.00,'batteryPenalty',0.10,'signalPenalty',500.00,...
    'dischargePenalty',2000.00,'maxDischargeRate',0.5);
ga.populationSize = 50; ga.maxGenerations = 20; ga.mutationRate = 0.1;
boa.populationSize = 20; boa.maxGenerations = 20; boa.fmin=0; boa.fmax=2; boa.loudness=0.5; boa.pulseRate=0.5;
R1 = 8; R2 = 10; k1 = 0.1; k2 = 100;
numNodes = 10;

% --- Environment Generation ---
% (This is the full environment generation code)
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
nodeRadius = 4.0;
truckNetworkNodes = zeros(numNodes, 2);
for i = 1:numNodes, angle = 2 * pi * i / numNodes; truckNetworkNodes(i, :) = centralPoint + nodeRadius * [cos(angle), sin(angle)]; end
depotLocation = [xOffset_truck, yOffset_truck + 5.0];


% =========================================================================
% SECTION 3: BI-LEVEL OPTIMIZATION (GA + BOA)
% =========================================================================
bestOverallCost = inf; bestSelectedSpots = []; 
bestCostHistory = zeros(ga.maxGenerations, 1); avgCostHistory = zeros(ga.maxGenerations, 1);

fprintf('Starting GA+BOA optimization...\n');
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
        for i = ga.populationSize/2 + 1 : ga.populationSize, if rand < ga.mutationRate, mutationPoints = randperm(numNodes, 2); temp = newPopulation(i, mutationPoints(1)); newPopulation(i, mutationPoints(1)) = newPopulation(i, mutationPoints(2)); newPopulation(i, mutationPoints(2)) = temp; end; end
        population = newPopulation;
    end
    costMatrix = zeros(numTrucks, ga.populationSize);
    parfor p = 1:ga.populationSize
        currentRouteIndices = population(p, :); currentCostForIndividual = zeros(numTrucks, 1); selectedParkingNodeIndices = currentRouteIndices(1:numTrucks);
        for t = 1:numTrucks
            parkingNodeIndex = selectedParkingNodeIndices(t); startNode = truckNetworkNodes(parkingNodeIndex, :); truckTarget = targetPositions(mod(t-1, size(targetPositions, 1)) + 1, :);
            [~, truckTravelDistance] = AStarPath(truckGridMap, depotLocation, startNode, truckDiagFlightEnabled, worldSizeX, worldSizeY, false);
            if isinf(truckTravelDistance), currentCostForIndividual(t) = 1e9; continue; end
            truckTravelTime = truckTravelDistance / truckSpeed;
            startNode_local = startNode - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone]; truckTarget_local = truckTarget - [xOffset_truck - xOffset_drone, yOffset_truck - yOffset_drone];
            [~, ~, ~, missionTime, ~] = runDroneBat(startNode_local, truckTarget_local, inflatedMap_drone, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, boa, [xOffset_drone, yOffset_drone], R1, R2, k1, k2, floorSizeX, floorSizeY);
            truckWaitTime = max(0, missionTime - truckTravelTime);
            totalMissionTime = truckTravelTime + truckWaitTime;
            if totalMissionTime > maxMissionTime, JTruck = 1e9;
            else, JTruck = (cost.truckPerKm * truckTravelDistance) + (cost.truckPerHour * truckWaitTime); end
            currentCostForIndividual(t) = JTruck;
        end
        costMatrix(:, p) = currentCostForIndividual;
    end
    totalCosts = sum(costMatrix, 1);
    bestCostHistory(gen) = min(totalCosts);
    avgCostHistory(gen) = mean(totalCosts);
    [minGenCost, bestIdx] = min(totalCosts);
    if minGenCost < bestOverallCost, bestOverallCost = minGenCost; bestRouteIndices = population(bestIdx, :); bestSelectedSpots = truckNetworkNodes(bestRouteIndices(1:numTrucks), :); end
    fprintf('Generation %d/%d: Best Cost = $%.2f\n', gen, ga.maxGenerations, bestCostHistory(gen));
end
parkingSpots = bestSelectedSpots;
overall_fleet_cost = bestOverallCost;

% ... (All plotting and saving sections from your final working script would go here) ...
% Remember to change save paths to use the 'save_dir' variable.

end