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