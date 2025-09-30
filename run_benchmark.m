% =========================================================================
% BENCHMARK SCRIPT FOR BI-LEVEL TRUCK-DRONE OPTIMIZATION (v4)
%
% This script uses an optimal "bag of tasks" parallel approach. It creates
% a list of all simulation jobs (scenario/algorithm/run) and executes
% them in a single parallel loop, ensuring no worker is idle.
%
% Author: Gemini
% Date:   September 29, 2025
% =========================================================================
clear; close all; clc;

%% ========================================================================
% SECTION 1: BENCHMARK CONFIGURATION
% =========================================================================

% --- General Settings ---
NUM_RUNS = 5; % Number of statistical runs
BASE_OUTPUT_DIR = 'Benchmark_Results_Optimal_Parallel';

% --- Algorithm Definitions ---
algorithms_to_test = {
    {'GA', 'BOA'},   {'GA', 'PSO'},   {'ACO', 'BSO'},...
    {'ALNS', 'PSO'}, {'ACO', 'CMAES'}
};

% --- Scenario Definitions ---
scenarios = struct();
scenarios(1).name = 'Scenario_1_Low_Complexity';
scenarios(1).numTargets = 5; scenarios(1).numObstacles = 25;
scenarios(2).name = 'Scenario_2_Medium_Complexity';
scenarios(2).numTargets = 8; scenarios(2).numObstacles = 40;
scenarios(3).name = 'Scenario_3_High_Complexity';
scenarios(3).numTargets = 10; scenarios(3).numObstacles = 60;

%% ========================================================================
% SECTION 2: CREATE THE "BAG OF TASKS"
% =========================================================================

fprintf('Creating the benchmark job list...\n');
job_list = {};
job_idx = 1;
for s_idx = 1:length(scenarios)
    for a_idx = 1:length(algorithms_to_test)
        for r_idx = 1:NUM_RUNS
            job.scenario_idx = s_idx;
            job.algo_idx = a_idx;
            job.run_idx = r_idx;
            job_list{job_idx} = job;
            job_idx = job_idx + 1;
        end
    end
end
total_jobs = length(job_list);
fprintf('%d total simulation jobs created.\n', total_jobs);

%% ========================================================================
% SECTION 3: PARALLEL EXECUTION
% =========================================================================

% --- Start Parallel Pool ---
if isempty(gcp('nocreate')), parpool('local'); else, disp('Parallel pool already running.'); end

fprintf('\nStarting parallel execution of all jobs...\n');

% Pre-allocate a results structure for the parfor loop
results_data = cell(total_jobs, 1);

parfor i = 1:total_jobs
    % --- Get job details from the list ---
    current_job = job_list{i};
    s_idx = current_job.scenario_idx;
    a_idx = current_job.algo_idx;
    run_idx = current_job.run_idx;
    
    scenario = scenarios(s_idx);
    algo_pair = algorithms_to_test{a_idx};
    algo_name = sprintf('%s_%s', algo_pair{1}, algo_pair{2});
    
    fprintf('Starting: [%s] - [%s] - Run %d\n', scenario.name, algo_name, run_idx);
    
    % --- Set up directories and parameters ---
    run_dir = fullfile(BASE_OUTPUT_DIR, scenario.name, algo_name, sprintf('Run_%02d', run_idx));
    if ~exist(run_dir, 'dir'), mkdir(run_dir); end
    
    sim_params = struct();
    sim_params.numTargets = scenario.numTargets;
    sim_params.numObstacles = scenario.numObstacles;
    sim_params.maxPackageWeight = 20.0;
    sim_params.lower_level_algo = algo_pair{1};
    sim_params.upper_level_algo = algo_pair{2};
    sim_params.rng_seed = s_idx * 1000 + a_idx * 100 + run_idx; % Deterministic seed
    sim_params.output_dir = run_dir;
    
    % --- Execute the simulation ---
    try
        [final_cost, comp_time] = runSingleSimulation(sim_params);
        results_data{i} = struct('scenario', scenario.name, 'algorithm', algo_name, 'cost', final_cost, 'time', comp_time);
        fprintf('Finished: [%s] - [%s] - Run %d | Cost: %.2f\n', scenario.name, algo_name, run_idx, final_cost);
    catch ME
        fprintf('FATAL ERROR in [%s] - [%s] - Run %d: %s\n', scenario.name, algo_name, run_idx, ME.message);
        results_data{i} = struct('scenario', scenario.name, 'algorithm', algo_name, 'cost', NaN, 'time', NaN);
    end
end

fprintf('\nAll parallel jobs complete.\n');

%% ========================================================================
% SECTION 4: FINAL REPORT GENERATION
% =========================================================================
fprintf('Generating final report...\n');
summary_file_path = fullfile(BASE_OUTPUT_DIR, 'benchmark_summary.csv');
results_table = table();

% Aggregate results by scenario and algorithm
unique_scenarios = {scenarios.name};
unique_algorithms = cellfun(@(c) sprintf('%s_%s', c{1}, c{2}), algorithms_to_test, 'UniformOutput', false);

for s_name = unique_scenarios
    for a_name = unique_algorithms
        costs = [];
        times = [];
        for i = 1:total_jobs
            res = results_data{i};
            if strcmp(res.scenario, s_name) && strcmp(res.algorithm, a_name)
                costs(end+1) = res.cost;
                times(end+1) = res.time;
            end
        end
        
        new_row = {s_name, a_name, mean(costs, 'omitnan'), std(costs, 'omitnan'), min(costs), max(costs), mean(times, 'omitnan')};
        results_table = [results_table; new_row];
    end
end

results_table.Properties.VariableNames = {'Scenario', 'Algorithm', 'Mean_Cost_USD', 'Std_Dev_Cost', 'Min_Cost_USD', 'Max_Cost_USD', 'Mean_Time_sec'};
disp(results_table);
writetable(results_table, summary_file_path);
fprintf('\nFinal report saved to:\n%s\n', summary_file_path);
delete(gcp('nocreate'));