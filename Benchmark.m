% =========================================================================
% BENCHMARK SCRIPT FOR BI-LEVEL TRUCK-DRONE OPTIMIZATION
%
% This script orchestrates a series of simulations to compare the
% performance of different bi-level heuristic pairs across various
% scenarios of increasing complexity.
% =========================================================================
clear; close all; clc;

%% --- BENCHMARK CONFIGURATION ---
scenarios = {
    struct('name', 'Scenario_1', 'numObstacles', 25, 'numTargets', 5, 'numTrucks', 5), ...
    struct('name', 'Scenario_2', 'numObstacles', 40, 'numTargets', 8, 'numTrucks', 8), ...
    struct('name', 'Scenario_3', 'numObstacles', 60, 'numTargets', 12, 'numTrucks', 12)
};

algorithms = {
    struct('name', 'GA_BOA', 'solver', @solve_with_GA_BOA), ...
    struct('name', 'ALNS_PSO', 'solver', @solve_with_ALNS_PSO), ...
    struct('name', 'ACO_CMAES', 'solver', @solve_with_ACO_CMAES)
};

numRepetitions = 1; % Number of runs for statistical significance
base_save_dir = 'Benchmark_Results';

% Initialize a table to store summary results
summary_results = table();

%% --- MAIN BENCHMARK LOOP ---
if isempty(gcp('nocreate')), parpool; end % Start parallel pool once

for s_idx = 1:length(scenarios)
    scenario = scenarios{s_idx};
    
    for a_idx = 1:length(algorithms)
        algorithm = algorithms{a_idx};
        
        fprintf('\n======================================================\n');
        fprintf('RUNNING: Scenario "%s" with Algorithm "%s"\n', scenario.name, algorithm.name);
        fprintf('======================================================\n');
        
        run_costs = zeros(numRepetitions, 1);
        
        for rep = 1:numRepetitions
            fprintf('\n--- Repetition %d of %d ---\n', rep, numRepetitions);
            
            % Create directories for saving results
            run_dir = fullfile(base_save_dir, scenario.name, algorithm.name, sprintf('Run_%d', rep));
            if ~exist(run_dir, 'dir'), mkdir(run_dir); end
            
            % Run the main simulation function
            try
                [final_fleet_cost, ~] = algorithm.solver( ...
                    scenario.numTrucks, ...
                    scenario.numTargets, ...
                    scenario.numObstacles, ...
                    run_dir ...
                );
                run_costs(rep) = final_fleet_cost;
            catch ME
                fprintf('!!! ERROR in simulation run: %s\n', ME.message);
                fprintf('Error in file: %s, line %d\n', ME.stack(1).file, ME.stack(1).line);
                run_costs(rep) = NaN; % Mark failed run
            end
        end
        
        % Calculate and store summary statistics
        avg_cost = nanmean(run_costs);
        std_dev = nanstd(run_costs);
        
        new_row = {scenario.name, algorithm.name, avg_cost, std_dev};
        summary_results = [summary_results; new_row];
        
        fprintf('\n--- SUMMARY for %s on %s ---\n', algorithm.name, scenario.name);
        fprintf('Average Fleet Cost: $%.2f\n', avg_cost);
        fprintf('Standard Deviation: $%.2f\n', std_dev);
        fprintf('-------------------------------------------------\n');
    end
end

%% --- FINAL REPORT GENERATION ---
summary_results.Properties.VariableNames = {'Scenario', 'Algorithm', 'AverageCost', 'StdDev'};
disp(summary_results);

% Write summary to a text file
summary_filename = fullfile(base_save_dir, 'benchmark_summary.txt');
writetable(summary_results, summary_filename, 'Delimiter', '\t');
fprintf('\nBenchmark complete. Summary report saved to %s\n', summary_filename);