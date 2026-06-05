% GPU-ACCELERATED PSO FOR DRONE OPTIMIZATION
function [gBestPos, gBestCost, costHistory, missionTime, costComponents] = runDronePSO_GPU(startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, pso, towerPosition, R1, R2, k1, k2, droneMapX, droneMapY)
    nVar = 3; varMin = gpuArray([0.5, 0.5, 0.5]); varMax = gpuArray([2.0, 2.0, 2.0]);
    costHistory = zeros(pso.maxGenerations, 1);
    
    [pathXY, ~] = AStarPath(gridMap, startPos, targetPos, droneDiagFlightEnabled, droneMapX, droneMapY, false);
    if isempty(pathXY), gBestPos = [1,1,1]; gBestCost = 1e10; costHistory(:)=gBestCost; missionTime = inf; costComponents=[inf,inf,inf,inf]; return; end
    
    % Initialize populations on GPU
    positions = varMin + (varMax - varMin) .* gpuArray.rand(pso.populationSize, nVar);
    velocities = gpuArray.zeros(pso.populationSize, nVar);
    costs = gpuArray.zeros(pso.populationSize, 1);
    
    % Vectorized fitness evaluation
    for i = 1:pso.populationSize
        costs(i) = JDroneCost(gather(positions(i,:)), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
    end
    
    pBestPos = positions;
    pBestCosts = costs;
    [gBestCost, gBestIdx] = min(costs);
    gBestPos_gpu = positions(gBestIdx, :);
    
    % PSO iterations with GPU
    for gen = 1:pso.maxGenerations
        % Vectorized velocity and position updates
        r1 = gpuArray.rand(pso.populationSize, nVar);
        r2 = gpuArray.rand(pso.populationSize, nVar);
        
        velocities = pso.inertia * velocities + ...
                     pso.cognition * r1 .* (pBestPos - positions) + ...
                     pso.social * r2 .* (gBestPos_gpu - positions);
        
        positions = positions + velocities;
        positions = max(positions, varMin);
        positions = min(positions, varMax);
        
        % Batch fitness evaluation
        for i = 1:pso.populationSize
            newCost = JDroneCost(gather(positions(i,:)), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
            if newCost < pBestCosts(i)
                pBestPos(i,:) = positions(i,:);
                pBestCosts(i) = newCost;
                if newCost < gBestCost
                    gBestCost = newCost;
                    gBestPos_gpu = positions(i,:);
                end
            end
        end
        
        costHistory(gen) = gather(gBestCost);
    end
    
    gBestPos = gather(gBestPos_gpu);
    gBestCost = gather(gBestCost);
    [gBestCost, missionTime, costComponents] = JDroneCost(gBestPos, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
end

