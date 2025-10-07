% =========================================================================
% BENCHMARK SCRIPT FOR BI-LEVEL TRUCK-DRONE OPTIMIZATION (v8 - Final)
%
% This final version integrates the Friedman and Wilcoxon statistical test
% results directly into the 'benchmark_summary.csv' output file for a
% complete, unified report.
%
% Author: Gemini
% Date:   October 01, 2025
% =========================================================================
clear; close all; clc;

benchmark_timer = tic;

%% SECTION 1: BENCHMARK CONFIGURATION
NUM_RUNS = 5;
BASE_OUTPUT_DIR = 'Benchmark_Results_Final';
algorithms_to_test = {
    {'GA', 'BOA'},   {'GA', 'PSO'},   {'ACO', 'BSO'},...
    {'ALNS', 'PSO'}, {'ACO', 'CMAES'}
};

scenarios = struct();
scenarios(1).name = 'Scenario_1_Low_Complexity'; scenarios(1).numTargets = 5; scenarios(1).numObstacles = 25;
scenarios(2).name = 'Scenario_2_Medium_Complexity'; scenarios(2).numTargets = 8; scenarios(2).numObstacles = 40;
scenarios(3).name = 'Scenario_3_High_Complexity'; scenarios(3).numTargets = 10; scenarios(3).numObstacles = 60;

%% SECTION 2: CREATE THE "BAG OF TASKS"
fprintf('Creating the benchmark job list...\n');
job_list = {}; job_idx = 1;
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

%% SECTION 3: PARALLEL EXECUTION
if isempty(gcp('nocreate')), parpool('local'); else, disp('Parallel pool already running.'); end
fprintf('\nStarting parallel execution of all jobs...\n');
results_data = cell(total_jobs, 1);
parfor i = 1:total_jobs
    current_job = job_list{i};
    s_idx = current_job.scenario_idx; a_idx = current_job.algo_idx; run_idx = current_job.run_idx;
    scenario = scenarios(s_idx);
    algo_pair = algorithms_to_test{a_idx};
    algo_name = sprintf('%s_%s', algo_pair{1}, algo_pair{2});
    
    fprintf('Starting: [%s] - [%s] - Run %d\n', scenario.name, algo_name, run_idx);
    
    run_dir = fullfile(BASE_OUTPUT_DIR, scenario.name, algo_name, sprintf('Run_%02d', run_idx));
    if ~exist(run_dir, 'dir'), mkdir(run_dir); end
    
    sim_params = struct();
    sim_params.numTargets = scenario.numTargets;
    sim_params.numObstacles = scenario.numObstacles;
    sim_params.maxPackageWeight = 20.0;
    sim_params.lower_level_algo = algo_pair{1};
    sim_params.upper_level_algo = algo_pair{2};
    sim_params.rng_seed = s_idx * 1000 + run_idx;
    sim_params.output_dir = run_dir;
    
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

%% SECTION 4: FINAL REPORT GENERATION
fprintf('Generating final report...\n');
if ~exist(BASE_OUTPUT_DIR, 'dir'), mkdir(BASE_OUTPUT_DIR); end
summary_file_path = fullfile(BASE_OUTPUT_DIR, 'benchmark_summary.csv');
results_table = table();
unique_scenarios = {scenarios.name};
unique_algorithms = cellfun(@(c) sprintf('%s_%s', c{1}, c{2}), algorithms_to_test, 'UniformOutput', false);

all_results_agg = struct();
for s_name_cell = unique_scenarios
    s_name = s_name_cell{:};
    for a_name_cell = unique_algorithms
        a_name = a_name_cell{:};
        costs = [];
        times = [];
        for i = 1:total_jobs
            res = results_data{i};
            if strcmp(res.scenario, s_name) && strcmp(res.algorithm, a_name)
                costs(end+1) = res.cost;
                times(end+1) = res.time;
            end
        end
        all_results_agg.(s_name).(a_name).costs = costs;
        
        new_row = {s_name, a_name, mean(costs, 'omitnan'), std(costs, 'omitnan'), min(costs), max(costs), mean(times, 'omitnan')};
        results_table = [results_table; new_row];
    end
end
results_table.Properties.VariableNames = {'Scenario', 'Algorithm', 'Mean_Cost_USD', 'Std_Dev_Cost', 'Min_Cost_USD', 'Max_Cost_USD', 'Mean_Time_sec'};

disp('======================================================');
disp('               Benchmark Summary Table                ');
disp('======================================================');
disp(results_table);

fprintf('Writing main summary table to: %s\n', summary_file_path);
writetable(results_table, summary_file_path);

%% SECTION 5: STATISTICAL ANALYSIS & APPEND TO CSV
fprintf('\nPerforming statistical analysis and appending to CSV...\n');

% Create empty cells for spacing in the CSV file
spacer = cell(1, width(results_table));

for s_name_cell = unique_scenarios
    s_name = s_name_cell{:};
    
    % Append a header for the scenario to the CSV
    scenario_header = {['STATISTICAL ANALYSIS FOR: ' s_name]};
    writecell(spacer, summary_file_path, 'WriteMode', 'append');
    writecell(scenario_header, summary_file_path, 'WriteMode', 'append');
    
    % --- Friedman Test ---
    cost_matrix = [];
    for a_name_cell = unique_algorithms
        a_name = a_name_cell{:};
        cost_matrix = [cost_matrix, all_results_agg.(s_name).(a_name).costs(:)];
    end
    
    p_friedman = NaN;
    try
        [p_friedman, ~, ~] = friedman(cost_matrix, 1, 'off');
    catch
        % Friedman test can fail if data has too many NaNs or ties
    end
    
    friedman_table = table({s_name}, p_friedman, 'VariableNames', {'Scenario', 'Friedman_p_value'});
    disp('--- Friedman Test ---');
    disp(friedman_table);
    writecell({'--- Friedman Test ---'}, summary_file_path, 'WriteMode', 'append');
    writetable(friedman_table, summary_file_path, 'WriteMode', 'append');
    
    % --- Wilcoxon Signed-Rank Test ---
    p_values = NaN(length(unique_algorithms));
    for i = 1:length(unique_algorithms)
        for j = i + 1:length(unique_algorithms)
            algo1_name = unique_algorithms{i};
            algo2_name = unique_algorithms{j};
            
            costs1 = all_results_agg.(s_name).(algo1_name).costs;
            costs2 = all_results_agg.(s_name).(algo2_name).costs;
            
            valid_pairs = ~isnan(costs1) & ~isnan(costs2);
            if sum(valid_pairs) >= 2 % Need at least 2 pairs to run the test
                p = signrank(costs1(valid_pairs), costs2(valid_pairs));
            else
                p = NaN;
            end
            p_values(i, j) = p;
            p_values(j, i) = p; % Make the table symmetric for easier reading
        end
    end
    
    wilcoxon_table = array2table(p_values, 'RowNames', unique_algorithms, 'VariableNames', unique_algorithms);
    disp('--- Wilcoxon Signed-Rank Test (p-values) ---');
    disp(wilcoxon_table);
    writecell(spacer, summary_file_path, 'WriteMode', 'append');
    writecell({'--- Wilcoxon Signed-Rank Test (p-values) ---'}, summary_file_path, 'WriteMode', 'append');
    writetable(wilcoxon_table, summary_file_path, 'WriteMode', 'append', 'WriteRowNames', true);
end

fprintf('\nStatistical analysis complete and appended to CSV.\n');

% --- Report Total Time ---
total_benchmark_time_sec = toc(benchmark_timer);
fprintf('\n------------------------------------------------------\n');
fprintf('Total Benchmark Execution Time: %.2f minutes.\n', total_benchmark_time_sec / 60);
fprintf('------------------------------------------------------\n');
delete(gcp('nocreate'));