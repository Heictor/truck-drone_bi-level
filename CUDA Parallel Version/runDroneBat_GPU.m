% GPU-ACCELERATED BAT ALGORITHM
function [bestParams, minCost, costHistory, missionTime, costComponents] = runDroneBat_GPU(startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, boa, towerPosition, R1, R2, k1, k2, droneMapX, droneMapY)
    nVar = 3; varMin = gpuArray([0.5, 0.5, 0.5]); varMax = gpuArray([2.0, 2.0, 2.0]);
    costHistory = zeros(boa.maxGenerations, 1);
    
    [pathXY, ~] = AStarPath(gridMap, startPos, targetPos, droneDiagFlightEnabled, droneMapX, droneMapY, false);
    if isempty(pathXY), minCost = 1e10; bestParams = [1, 1, 1]; costHistory(:)=minCost; missionTime = inf; costComponents=[inf,inf,inf,inf]; return; end
    
    % Initialize on GPU
    positions = varMin + (varMax - varMin) .* gpuArray.rand(boa.populationSize, nVar);
    velocities = gpuArray.zeros(boa.populationSize, nVar);
    loudness = gpuArray(boa.loudness * ones(boa.populationSize, 1));
    pulseRates = gpuArray(boa.pulseRate * ones(boa.populationSize, 1));
    costs = gpuArray.zeros(boa.populationSize, 1);
    
    for i = 1:boa.populationSize
        costs(i) = JDroneCost(gather(positions(i,:)), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
    end
    
    [minCost, bestIdx] = min(costs);
    bestParams_gpu = positions(bestIdx, :);
    
    stagnation_counter = 0;
    for gen = 1:boa.maxGenerations
        last_min_cost = minCost;
        
        % Vectorized bat algorithm updates
        frequencies = gpuArray(boa.fmin + (boa.fmax - boa.fmin) * rand(boa.populationSize, 1));
        velocities = velocities + (positions - bestParams_gpu) .* frequencies;
        newPositions = positions + velocities;
        newPositions = max(newPositions, varMin);
        newPositions = min(newPositions, varMax);
        
        % Random walk for some bats
        randomWalk = gpuArray.rand(boa.populationSize, 1) > pulseRates;
        walkPositions = bestParams_gpu + 0.1 * gpuArray.randn(boa.populationSize, nVar);
        walkPositions = max(walkPositions, varMin);
        walkPositions = min(walkPositions, varMax);
        newPositions(randomWalk, :) = walkPositions(randomWalk, :);
        
        % Evaluate and update
        for i = 1:boa.populationSize
            newCost = JDroneCost(gather(newPositions(i,:)), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
            if newCost < costs(i) && rand < loudness(i)
                positions(i,:) = newPositions(i,:);
                costs(i) = newCost;
                loudness(i) = loudness(i) * 0.9;
                pulseRates(i) = boa.pulseRate * (1 - exp(-gen/boa.maxGenerations));
            end
        end
        
        [minCost, bestIdx] = min(costs);
        bestParams_gpu = positions(bestIdx, :);
        
        if minCost >= last_min_cost
            stagnation_counter = stagnation_counter + 1;
        else
            stagnation_counter = 0;
        end
        
        % Diversification
        if stagnation_counter >= 7
            [~, sortOrder] = sort(costs, 'descend');
            numReinit = floor(boa.populationSize/2);
            positions(sortOrder(1:numReinit),:) = varMin + (varMax - varMin) .* gpuArray.rand(numReinit, nVar);
            for i = sortOrder(1:numReinit)'
                costs(i) = JDroneCost(gather(positions(i,:)), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
            end
            stagnation_counter = 0;
        end
        
        costHistory(gen) = gather(minCost);
    end
    
    bestParams = gather(bestParams_gpu);
    minCost = gather(minCost);
    [minCost, missionTime, costComponents] = JDroneCost(bestParams, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
end


