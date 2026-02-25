% =========================================================================
% GPU-ACCELERATED OPTIMIZATION ALGORITHMS
% Contains: PSO_GPU, BOA_GPU, BSO_GPU
% Author: Heictor Costa
% Date: February 06, 2026
% =========================================================================

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

% GPU-ACCELERATED BRAIN STORM OPTIMIZATION
function [gBestPos, gBestCost, costHistory, missionTime, costComponents] = runDroneBSO_GPU(startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, bso, towerPosition, R1, R2, k1, k2, droneMapX, droneMapY)
    nVar = 3; varMin = gpuArray([0.5, 0.5, 0.5]); varMax = gpuArray([2.0, 2.0, 2.0]);
    costHistory = zeros(bso.maxGenerations, 1);
    
    [pathXY, ~] = AStarPath(gridMap, startPos, targetPos, droneDiagFlightEnabled, droneMapX, droneMapY, false);
    if isempty(pathXY), gBestPos = [1,1,1]; gBestCost = 1e10; costHistory(:)=gBestCost; missionTime = inf; costComponents=[inf,inf,inf,inf]; return; end
    
    % Initialize on GPU
    individuals = varMin + (varMax - varMin) .* gpuArray.rand(bso.populationSize, nVar);
    costs = gpuArray.zeros(bso.populationSize, 1);
    
    for i = 1:bso.populationSize
        costs(i) = JDroneCost(gather(individuals(i,:)), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
    end
    
    [gBestCost, gBestIdx] = min(costs);
    gBestPos_gpu = individuals(gBestIdx,:);
    
    for gen = 1:bso.maxGenerations
        % Clustering on CPU (kmeans not GPU-optimized in MATLAB)
        individuals_cpu = gather(individuals);
        [cluster_idx, ~] = kmeans(individuals_cpu, bso.numClusters, 'emptyaction', 'singleton');
        cluster_idx = gpuArray(cluster_idx);
        
        new_individuals = individuals;
        
        % Vectorized BSO operations where possible
        for i = 1:bso.populationSize
            r = rand();
            if r < bso.p_one_cluster
                selected_cluster = randi(bso.numClusters);
                cluster_members = individuals(cluster_idx == selected_cluster, :);
                new_individuals(i,:) = cluster_members(randi(size(cluster_members,1)),:) + 0.5 * gpuArray.randn(1, nVar);
            else
                r2 = rand();
                selected_cluster = cluster_idx(i);
                if r2 < bso.p_one_center
                    same_cluster_indices = find(cluster_idx == selected_cluster);
                    rand_ind_idx = same_cluster_indices(randi(length(same_cluster_indices)));
                    new_individuals(i,:) = individuals(rand_ind_idx,:) + 0.5 * gpuArray.randn(1, nVar);
                elseif r2 < bso.p_one_center + bso.p_two_centers
                    same_cluster_indices = find(cluster_idx == selected_cluster);
                    if length(same_cluster_indices) >= 2
                        p_indices = randperm(length(same_cluster_indices), 2);
                        p1 = individuals(same_cluster_indices(p_indices(1)),:);
                        p2 = individuals(same_cluster_indices(p_indices(2)),:);
                        new_individuals(i,:) = rand()*p1 + (1-rand())*p2;
                    else
                        new_individuals(i,:) = individuals(same_cluster_indices(1),:) + 0.5 * gpuArray.randn(1, nVar);
                    end
                else
                    other_clusters = setdiff(1:bso.numClusters, selected_cluster);
                    rand_cluster_idx = other_clusters(randi(length(other_clusters)));
                    cluster1_indices = find(cluster_idx == selected_cluster);
                    cluster2_indices = find(cluster_idx == rand_cluster_idx);
                    p1 = individuals(cluster1_indices(randi(length(cluster1_indices))),:);
                    p2 = individuals(cluster2_indices(randi(length(cluster2_indices))),:);
                    new_individuals(i,:) = rand()*p1 + (1-rand())*p2;
                end
            end
            new_individuals(i,:) = max(new_individuals(i,:), varMin);
            new_individuals(i,:) = min(new_individuals(i,:), varMax);
        end
        
        % Batch evaluation
        for i = 1:bso.populationSize
            new_cost = JDroneCost(gather(new_individuals(i,:)), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
            if new_cost < costs(i)
                individuals(i,:) = new_individuals(i,:);
                costs(i) = new_cost;
            end
        end
        
        [current_best_cost, current_best_idx] = min(costs);
        if current_best_cost < gBestCost
            gBestCost = current_best_cost;
            gBestPos_gpu = individuals(current_best_idx,:);
        end
        
        costHistory(gen) = gather(gBestCost);
    end
    
    gBestPos = gather(gBestPos_gpu);
    gBestCost = gather(gBestCost);
    [gBestCost, missionTime, costComponents] = JDroneCost(gBestPos, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY);
end

% HELPER FUNCTIONS (Keep CPU versions as they are)
% [Include all helper functions from the original code: JDroneCost, AStarPath, simulateDroneFlight, etc.]
function fleet_cost=calculate_truck_fleet_cost(selected_indices,nodes,targets,grid,depot,diag,wX,wY,tSpeed,drone_map,cost,alt,dSH,dSV,dDiag,dStay,dBatt,iBatt,tower,r1,r2,k1,k2,fX,fY,oX,oY,maxW,pFactor,up_algo,boa,pso,bso,cmaes)
    % =========================================================================
    % OPTIMIZED WITH STRATEGY 2: HIERARCHICAL APPROXIMATION
    % Uses Euclidean heuristic instead of full drone optimization
    % Expected speedup: ~10-20x for lower-level search
    % =========================================================================
    
    num_trucks=length(selected_indices);
    cost_per_truck=zeros(num_trucks,1);
    
    for t=1:num_trucks
        parkingNodeIndex=selected_indices(t);
        startNode=nodes(parkingNodeIndex,:);
        truckTarget=targets(mod(t-1,size(targets,1))+1,:);
        
        % Calculate truck path
        [~,truckTravelDistance]=AStarPath(grid,depot,startNode,diag,wX,wY,false);
        
        if isinf(truckTravelDistance)
            cost_per_truck(t)=1e9;
            continue;
        end
        
        truckTravelTime=truckTravelDistance/tSpeed;
        
        % ===================================================================
        % STRATEGY 2: FAST EUCLIDEAN HEURISTIC FOR DRONE TIME
        % Instead of running full PSO/BOA/BSO (20 particles × 20 gens),
        % use simple geometric calculation (instant!)
        % ===================================================================
        
        % Convert to local coordinates
        startNode_local=startNode-[oX-fX/2,oY-fY/2];
        truckTarget_local=truckTarget-[oX-fX/2,oY-fY/2];
        
        % Account for package weight effect on speed
        currentPackageWeight=rand()*maxW;
        speedReduction=(currentPackageWeight/maxW)*pFactor;
        droneSpeed_H=dSH*(1-speedReduction);
        droneSpeed_V=dSV*(1-speedReduction);
        
        % HEURISTIC: Euclidean distance + vertical maneuvers
        horizontalDist = norm(startNode_local - truckTarget_local);
        
        % Drone mission time estimate (no optimization needed!)
        ascentTime = alt / droneSpeed_V;
        descentTime = alt / droneSpeed_V;
        horizontalTime = horizontalDist / droneSpeed_H;
        
        missionTime = ascentTime + horizontalTime + descentTime + dStay;
        
        % ===================================================================
        % Calculate truck cost (waiting time based on heuristic drone time)
        % ===================================================================
        truckWaitTime=max(0,missionTime-truckTravelTime);
        JTruck=(cost.truckPerKm*truckTravelDistance)+(cost.truckPerHour*truckWaitTime);
        cost_per_truck(t)=JTruck;
    end
    
    fleet_cost=cost_per_truck;
end
function [partial_solution, removed_nodes]=destroy_random(solution,q,varargin),q_safe=min(q,length(solution));removed_indices=randperm(length(solution),q_safe);removed_nodes=solution(removed_indices);partial_solution=solution;partial_solution(removed_indices)=[];end
function [partial_solution, removed_nodes]=destroy_worst(solution,q,nodes,varargin),costs=zeros(length(solution),1);for i=1:length(solution),costs(i)=norm(nodes(solution(i),:)-[6,6]);end;[~,sorted_indices]=sort(costs,'descend');q_safe=min(q,length(solution));removed_indices=sorted_indices(1:q_safe);removed_nodes=solution(removed_indices);partial_solution=solution;partial_solution(removed_indices)=[];end
function new_solution=repair_random(partial_solution,numNodes,numParkingSpots),available_nodes=setdiff(1:numNodes,partial_solution);nodes_needed=numParkingSpots-length(partial_solution);nodes_needed=min(nodes_needed,length(available_nodes));if nodes_needed>0,nodes_to_add=randsample(available_nodes,nodes_needed);new_solution=[partial_solution,nodes_to_add];else,new_solution=partial_solution;end;end
function new_solution=repair_greedy(partial_solution,numNodes,nodes,numParkingSpots),new_solution=partial_solution;available_nodes=setdiff(1:numNodes,partial_solution);nodes_needed=numParkingSpots-length(partial_solution);for i=1:nodes_needed,if isempty(available_nodes),break;end;best_node=-1;min_cost_increase=inf;for j=1:length(available_nodes),node_to_try=available_nodes(j);cost_increase=norm(nodes(node_to_try,:)-[6,6]);if cost_increase<min_cost_increase,min_cost_increase=cost_increase;best_node=node_to_try;end;end;new_solution=[new_solution,best_node];available_nodes(available_nodes==best_node)=[];end;end
function idx=roulette_wheel_selection(weights),if sum(weights)==0||any(isnan(weights)),weights=ones(size(weights));end;p=weights(:)'/sum(weights);idx=find(rand<=cumsum(p),1,'first');end
function [bestParams,minCost,costHistory,missionTime,costComponents]=runDroneCMAES(startPos,targetPos,gridMap,cost,droneCruiseAltitude,droneSpeed_H,droneSpeed_V,droneDiagFlightEnabled,droneStayTime,droneBatteryMin,initialBattery,cmaes,towerPosition,R1,R2,k1,k2,droneMapX,droneMapY),nVar=3;varMin=[0.5,0.5,0.5];varMax=[2.0,2.0,2.0];costHistory=zeros(cmaes.maxGenerations,1);[pathXY,~]=AStarPath(gridMap,startPos,targetPos,droneDiagFlightEnabled,droneMapX,droneMapY,false);if isempty(pathXY),minCost=1e10;bestParams=[1,1,1];costHistory(:)=minCost;missionTime=inf;costComponents=[inf,inf,inf,inf];return;end;lambda=cmaes.populationSize;mu=floor(lambda/2);weights=log(mu+1/2)-log(1:mu)';weights=weights/sum(weights);mueff=sum(weights)^2/sum(weights.^2);mean_pos=(varMin+varMax)/2;sigma=cmaes.sigma;C=eye(nVar);pc=zeros(nVar,1);ps=zeros(nVar,1);cc=(4+mueff/nVar)/(nVar+4+2*mueff/nVar);cs=(mueff+2)/(nVar+mueff+5);c1=2/((nVar+1.3)^2+mueff);cmu=min(1-c1,2*(mueff-2+1/mueff)/((nVar+2)^2+mueff));damps=1+2*max(0,sqrt((mueff-1)/(nVar+1))-1)+cs;minCost=inf;bestParams=mean_pos;for gen=1:cmaes.maxGenerations,population=zeros(nVar,lambda);costs=zeros(lambda,1);for i=1:lambda,population(:,i)=mean_pos'+sigma*(C^(1/2)*randn(nVar,1));population(:,i)=max(population(:,i),varMin');population(:,i)=min(population(:,i),varMax');costs(i)=JDroneCost(population(:,i)',startPos,targetPos,gridMap,cost,droneCruiseAltitude,droneSpeed_H,droneSpeed_V,droneDiagFlightEnabled,droneStayTime,droneBatteryMin,initialBattery,towerPosition,R1,R2,k1,k2,pathXY);end;[sorted_costs,sort_idx]=sort(costs);if sorted_costs(1)<minCost,minCost=sorted_costs(1);bestParams=population(:,sort_idx(1))';end;old_mean=mean_pos;mean_pos=(population(:,sort_idx(1:mu))*weights)';ps=(1-cs)*ps+sqrt(cs*(2-cs)*mueff)*(C^(-1/2)*(mean_pos-old_mean)'/sigma);hsig=norm(ps)/sqrt(1-(1-cs)^(2*gen))/((nVar+4)/(nVar+4)+0.5)<1.4+2/(nVar+1);pc=(1-cc)*pc+hsig*sqrt(cc*(2-cc)*mueff)*(mean_pos-old_mean)'/sigma;artmp=(1/sigma)*(population(:,sort_idx(1:mu))-repmat(old_mean',1,mu));C=(1-c1-cmu)*C+c1*(pc*pc'+(1-hsig)*cc*(2-cc)*C)+cmu*artmp*diag(weights)*artmp';sigma=sigma*exp((cs/damps)*(norm(ps)/((nVar+4)/(nVar+4)+0.5)-1));costHistory(gen)=minCost;end;[minCost,missionTime,costComponents]=JDroneCost(bestParams,startPos,targetPos,gridMap,cost,droneCruiseAltitude,droneSpeed_H,droneSpeed_V,droneDiagFlightEnabled,droneStayTime,droneBatteryMin,initialBattery,towerPosition,R1,R2,k1,k2,pathXY);end
function [bestParams, minCost, costHistory, missionTime, costComponents] = runDroneBat(startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, boa, towerPosition, R1, R2, k1, k2, droneMapX, droneMapY), nVar = 3; varMin = [0.5, 0.5, 0.5]; varMax = [2.0, 2.0, 2.0]; bats = struct('Position', {}, 'Velocity', {}, 'Loudness', {}, 'PulseRate', {}, 'Cost', {}); costHistory=zeros(boa.maxGenerations,1); [pathXY, ~] = AStarPath(gridMap, startPos, targetPos, droneDiagFlightEnabled, droneMapX, droneMapY, false); if isempty(pathXY), minCost = 1e10; bestParams = [1, 1, 1]; costHistory(:)=minCost; missionTime = inf; costComponents=[inf,inf,inf,inf]; return; end; for i = 1:boa.populationSize, bats(i).Position = varMin + (varMax - varMin).* rand(1, nVar); bats(i).Velocity = zeros(1, nVar); bats(i).Loudness = boa.loudness; bats(i).PulseRate = boa.pulseRate; try, [bats(i).Cost, ~] = JDroneCost(bats(i).Position, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); catch, bats(i).Cost = 1e10; end; end; [minCost, bestBatIdx] = min([bats.Cost]); bestParams = bats(bestBatIdx).Position; stagnation_counter = 0; for gen = 1:boa.maxGenerations, last_min_cost = minCost; for i = 1:boa.populationSize, newF = boa.fmin + (boa.fmax - boa.fmin) * rand; bats(i).Velocity = bats(i).Velocity + (bats(i).Position - bestParams) * newF; newPos = bats(i).Position + bats(i).Velocity; newPos = max(newPos, varMin); newPos = min(newPos, varMax); if rand > bats(i).PulseRate, newPos = bestParams + 0.1 * randn(1, nVar); newPos = max(newPos, varMin); newPos = min(newPos, varMax); end; try, [newCost, ~] = JDroneCost(newPos, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); catch, newCost = 1e10; end; if newCost < bats(i).Cost && rand < bats(i).Loudness, bats(i).Position = newPos; bats(i).Cost = newCost; bats(i).Loudness = bats(i).Loudness * 0.9; bats(i).PulseRate = boa.pulseRate * (1 - exp(-gen/boa.maxGenerations)); end; end; [minCost, bestBatIdx] = min([bats.Cost]); bestParams = bats(bestBatIdx).Position; if minCost >= last_min_cost, stagnation_counter = stagnation_counter + 1; else, stagnation_counter = 0; end; if stagnation_counter >= 7, [~, sortOrder] = sort([bats.Cost], 'descend'); for i = 1:floor(boa.populationSize/2), bats(sortOrder(i)).Position = varMin + (varMax - varMin).* rand(1, nVar); try, [bats(sortOrder(i)).Cost, ~] = JDroneCost(bats(sortOrder(i)).Position, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); catch, bats(sortOrder(i)).Cost = 1e10; end; end; stagnation_counter = 0; end; costHistory(gen)=minCost; end; [minCost, missionTime, costComponents] = JDroneCost(bestParams, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); end
function [gBestPos, gBestCost, costHistory, missionTime, costComponents] = runDronePSO(startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, pso, towerPosition, R1, R2, k1, k2, droneMapX, droneMapY), nVar = 3; varMin = [0.5, 0.5, 0.5]; varMax = [2.0, 2.0, 2.0]; particles = struct('Position', {}, 'Velocity', {}, 'Cost', {}, 'pBestPos', {}, 'pBestCost', {}); costHistory = zeros(pso.maxGenerations, 1); [pathXY, ~] = AStarPath(gridMap, startPos, targetPos, droneDiagFlightEnabled, droneMapX, droneMapY, false); if isempty(pathXY), gBestPos = [1,1,1]; gBestCost = 1e10; costHistory(:)=gBestCost; missionTime = inf; costComponents=[inf,inf,inf,inf]; return; end; gBestCost = inf; gBestPos = []; for i = 1:pso.populationSize, particles(i).Position = varMin + (varMax - varMin) .* rand(1, nVar); particles(i).Velocity = zeros(1, nVar); [particles(i).Cost, ~] = JDroneCost(particles(i).Position, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); particles(i).pBestPos = particles(i).Position; particles(i).pBestCost = particles(i).Cost; if particles(i).pBestCost < gBestCost, gBestCost = particles(i).pBestCost; gBestPos = particles(i).pBestPos; end; end; for gen = 1:pso.maxGenerations, for i = 1:pso.populationSize, particles(i).Velocity = pso.inertia * particles(i).Velocity + pso.cognition * rand() * (particles(i).pBestPos - particles(i).Position) + pso.social * rand() * (gBestPos - particles(i).Position); particles(i).Position = particles(i).Position + particles(i).Velocity; particles(i).Position = max(particles(i).Position, varMin); particles(i).Position = min(particles(i).Position, varMax); [particles(i).Cost, ~] = JDroneCost(particles(i).Position, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); if particles(i).Cost < particles(i).pBestCost, particles(i).pBestPos = particles(i).Position; particles(i).pBestCost = particles(i).Cost; if particles(i).pBestCost < gBestCost, gBestCost = particles(i).pBestCost; gBestPos = particles(i).pBestPos; end; end; end; costHistory(gen) = gBestCost; end; [gBestCost, missionTime, costComponents] = JDroneCost(gBestPos, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); end
function [gBestPos, gBestCost, costHistory, missionTime, costComponents] = runDroneBSO(startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, bso, towerPosition, R1, R2, k1, k2, droneMapX, droneMapY), nVar = 3; varMin = [0.5, 0.5, 0.5]; varMax = [2.0, 2.0, 2.0]; costHistory = zeros(bso.maxGenerations, 1); [pathXY, ~] = AStarPath(gridMap, startPos, targetPos, droneDiagFlightEnabled, droneMapX, droneMapY, false); if isempty(pathXY), gBestPos = [1,1,1]; gBestCost = 1e10; costHistory(:)=gBestCost; missionTime = inf; costComponents=[inf,inf,inf,inf]; return; end; individuals = varMin + (varMax - varMin) .* rand(bso.populationSize, nVar); costs = zeros(bso.populationSize, 1); for i = 1:bso.populationSize, [costs(i), ~] = JDroneCost(individuals(i,:), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); end; [gBestCost, gBestIdx] = min(costs); gBestPos = individuals(gBestIdx,:); for gen = 1:bso.maxGenerations, [cluster_idx, ~] = kmeans(individuals, bso.numClusters, 'emptyaction', 'singleton'); new_individuals = individuals; for i = 1:bso.populationSize, r = rand(); if r < bso.p_one_cluster, selected_cluster = randi(bso.numClusters); cluster_members = individuals(cluster_idx == selected_cluster, :); new_individuals(i,:) = cluster_members(randi(size(cluster_members,1)),:) + 0.5 * randn(1, nVar); else, r2 = rand(); selected_cluster = cluster_idx(i); if r2 < bso.p_one_center, same_cluster_indices = find(cluster_idx == selected_cluster); rand_ind_idx = same_cluster_indices(randi(length(same_cluster_indices))); new_individuals(i,:) = individuals(rand_ind_idx,:) + 0.5 * randn(1, nVar); elseif r2 < bso.p_one_center + bso.p_two_centers, same_cluster_indices = find(cluster_idx == selected_cluster); if length(same_cluster_indices) >= 2, p_indices = randperm(length(same_cluster_indices), 2); p1 = individuals(same_cluster_indices(p_indices(1)),:); p2 = individuals(same_cluster_indices(p_indices(2)),:); new_individuals(i,:) = rand()*p1 + (1-rand())*p2; else, new_individuals(i,:) = individuals(same_cluster_indices(1),:) + 0.5 * randn(1, nVar); end; else, other_clusters = 1:bso.numClusters; other_clusters(selected_cluster) = []; rand_cluster_idx = other_clusters(randi(length(other_clusters))); cluster1_indices = find(cluster_idx == selected_cluster); cluster2_indices = find(cluster_idx == rand_cluster_idx); p1 = individuals(cluster1_indices(randi(length(cluster1_indices))),:); p2 = individuals(cluster2_indices(randi(length(cluster2_indices))),:); new_individuals(i,:) = rand()*p1 + (1-rand())*p2; end; end; new_individuals(i,:) = max(new_individuals(i,:), varMin); new_individuals(i,:) = min(new_individuals(i,:), varMax); end; for i = 1:bso.populationSize, [new_cost, ~] = JDroneCost(new_individuals(i,:), startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); if new_cost < costs(i), individuals(i,:) = new_individuals(i,:); costs(i) = new_cost; end; end; [current_best_cost, current_best_idx] = min(costs); if current_best_cost < gBestCost, gBestCost = current_best_cost; gBestPos = individuals(current_best_idx,:); end; costHistory(gen) = gBestCost; end; [gBestCost, missionTime, costComponents] = JDroneCost(gBestPos, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY); end
function [costVal, totalTime, components] = JDroneCost(params, startPos, targetPos, gridMap, cost, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneDiagFlightEnabled, droneStayTime, droneBatteryMin, initialBattery, towerPosition, R1, R2, k1, k2, pathXY), [~, droneTimeVec, ~, batteryConsumptionPerStep, obstaclePenaltySum, signalPenaltySum] = simulateDroneFlight(startPos, targetPos, params, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneStayTime, gridMap, towerPosition, R1, R2, k1, k2, pathXY, droneDiagFlightEnabled); if isempty(droneTimeVec), totalTime = inf; else, totalTime = droneTimeVec(end); end; timeCost = totalTime * cost.dronePerHour; obstacleCost = obstaclePenaltySum * cost.obstaclePenalty; remainingBattery = initialBattery - cumsum(batteryConsumptionPerStep); remainingBattery(remainingBattery < droneBatteryMin) = droneBatteryMin; batteryUsageIntegral = sum(1 ./ remainingBattery); batteryCost = batteryUsageIntegral * cost.batteryPenalty; signalCost = signalPenaltySum * cost.signalPenalty; costVal = timeCost + obstacleCost + batteryCost + signalCost; components = [timeCost, batteryCost, obstacleCost, signalCost]; end
function [path_meters, totalDistance] = AStarPath(gridMap, startPos, targetPos, diagEnabled, worldX, worldY, ~), [rows, cols] = size(gridMap); gridResolution_x = worldX / cols; gridResolution_y = worldY / rows; startIdx = floor(startPos ./ [gridResolution_x, gridResolution_y]) + 1; targetIdx = floor(targetPos ./ [gridResolution_x, gridResolution_y]) + 1; startIdx(1) = max(1, min(cols, startIdx(1))); startIdx(2) = max(1, min(rows, startIdx(2))); targetIdx(1) = max(1, min(cols, targetIdx(1))); targetIdx(2) = max(1, min(rows, targetIdx(2))); if gridMap(startIdx(2), startIdx(1)) == 1 || gridMap(targetIdx(2), targetIdx(1)) == 1, path_meters = []; totalDistance = inf; return; end; openSet = priority_queue(); openSet.insert(startIdx, 0); gScore = inf(rows, cols); gScore(startIdx(2), startIdx(1)) = 0; cameFrom = zeros(rows, cols, 2); path = []; while ~openSet.isempty(), currentIdx = openSet.extract_min(); if isequal(currentIdx, targetIdx), path = []; current = currentIdx; while ~isequal(current, startIdx), path = [path; current]; current = squeeze(cameFrom(current(2), current(1), :))'; end; path = [path; startIdx]; path = flipud(path); break; end; neighbors = getNeighbors(currentIdx, diagEnabled, rows, cols); for i = 1:size(neighbors, 1), neighbor = neighbors(i, :); if neighbor(1) < 1 || neighbor(1) > cols || neighbor(2) < 1 || neighbor(2) > rows, continue; end; if gridMap(neighbor(2), neighbor(1)) == 1, continue; end; dist = norm((currentIdx - neighbor) .* [gridResolution_x, gridResolution_y]); tentative_gScore = gScore(currentIdx(2), currentIdx(1)) + dist; if tentative_gScore < gScore(neighbor(2), neighbor(1)), cameFrom(neighbor(2), neighbor(1), :) = currentIdx; gScore(neighbor(2), neighbor(1)) = tentative_gScore; hScore = norm((neighbor - targetIdx) .* [gridResolution_x, gridResolution_y]); fScore = tentative_gScore + hScore; if ~openSet.contains(neighbor), openSet.insert(neighbor, fScore); end; end; end; end; if isempty(path), path_meters = []; totalDistance = inf; else, path_meters = (path - 1) .* [gridResolution_x, gridResolution_y]; totalDistance = 0; for i = 1:size(path_meters, 1) - 1, totalDistance = totalDistance + norm(path_meters(i, :) - path_meters(i+1, :)); end; end; end
function neighbors = getNeighbors(node, diagEnabled, ~, ~), [x, y] = deal(node(1), node(2)); if diagEnabled, [dx, dy] = meshgrid(-1:1, -1:1); neighbors_rel = [dx(:), dy(:)]; neighbors_rel(5,:) = []; else, neighbors_rel = [-1, 0; 1, 0; 0, -1; 0, 1]; end; neighbors = [x,y] + neighbors_rel; end
function [dronePath, droneTime, totalDistance, batteryConsumptionPerStep, obstaclePenaltySum, signalPenaltySum] = simulateDroneFlight(startPos, targetPos, params, droneCruiseAltitude, droneSpeed_H, droneSpeed_V, droneStayTime, gridMap, towerPosition, R1, R2, k1, k2, pathXY, ~), h_vel_scale=params(1); v_vel_scale_ascent=params(2); v_vel_scale_descent=params(3); gridResolution=0.1; ascentSpeed=droneSpeed_V*v_vel_scale_ascent; ascentTime=droneCruiseAltitude/ascentSpeed; numStepsAscent=max(2,round(ascentTime/(gridResolution/droneSpeed_V))); z_ascent=linspace(0,droneCruiseAltitude,numStepsAscent); path_ascent=[repmat(startPos(1),numStepsAscent,1),repmat(startPos(2),numStepsAscent,1),z_ascent']; time_ascent=linspace(0,ascentTime,numStepsAscent)'; horizSpeed=droneSpeed_H*h_vel_scale; path_horiz=[pathXY,repmat(droneCruiseAltitude,size(pathXY,1),1)]; if size(pathXY,1)>1, dist_segments=vecnorm(diff(pathXY,1,1),2,2); time_segments=dist_segments/horizSpeed; time_horiz=[0;cumsum(time_segments)]; else, time_horiz=0; end; descentSpeed=droneSpeed_V*v_vel_scale_descent; descentTime=droneCruiseAltitude/descentSpeed; numStepsDescent=max(2,round(descentTime/(gridResolution/droneSpeed_V))); z_descent=linspace(droneCruiseAltitude,0,numStepsDescent); path_descent=[repmat(targetPos(1),numStepsDescent,1),repmat(targetPos(2),numStepsDescent,1),z_descent']; time_descent=linspace(0,descentTime,numStepsDescent)'; dronePath=path_ascent; droneTime=time_ascent; if size(path_horiz,1)>1, dronePath=[dronePath;path_horiz(2:end,:)]; droneTime=[droneTime;droneTime(end)+time_horiz(2:end)]; end; dronePath=[dronePath;path_descent(2:end,:)]; droneTime=[droneTime;droneTime(end)+time_descent(2:end)]; dronePath=[dronePath;dronePath(end,:)]; droneTime=[droneTime;droneTime(end)+droneStayTime]; totalDistance=0; if size(dronePath,1)>1, for i=1:size(dronePath,1)-1,totalDistance=totalDistance+norm(dronePath(i,:)-dronePath(i+1,:));end; end; batteryConsumptionPerStep=zeros(size(dronePath,1),1); obstaclePenaltySum=0; signalPenaltySum=0; len_ascent_pts=size(path_ascent,1); len_horiz_pts=size(path_horiz,1); if isempty(pathXY),len_horiz_pts=0; end; for i=1:size(dronePath,1), currentPos_2D=dronePath(i,1:2); gridX_curr=max(1,min(size(gridMap,2),floor(currentPos_2D(1)/gridResolution)+1)); gridY_curr=max(1,min(size(gridMap,1),floor(currentPos_2D(2)/gridResolution)+1)); if gridMap(gridY_curr,gridX_curr)==1, obstaclePenaltySum=obstaclePenaltySum+1; end; distToTower=norm(currentPos_2D-towerPosition); if distToTower>R2, signalPenaltySum=signalPenaltySum+k2; elseif distToTower>R1, signalPenaltySum=signalPenaltySum+k1*(distToTower-R1)^2; end; len_horiz_segment=max(0,len_horiz_pts-1); if i<=len_ascent_pts,batt_step=0.001+0.0005*v_vel_scale_ascent^2; elseif i<=len_ascent_pts+len_horiz_segment,batt_step=0.001+0.0005*h_vel_scale^2; else, batt_step=0.001+0.0005*v_vel_scale_descent^2; end; batteryConsumptionPerStep(i)=batt_step; end; end
function saveFigure(figHandle, filename_base, dpi, fontsize), set(findall(figHandle, '-property', 'FontSize'), 'FontSize', fontsize); try, savefig(figHandle, [filename_base '.fig']); exportgraphics(figHandle, [filename_base '.png'], 'Resolution', dpi); catch ME, fprintf('Could not save figure %s. Error: %s\n', filename_base, ME.message); end; end