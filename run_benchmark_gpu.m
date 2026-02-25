% =========================================================================
% GPU-ACCELERATED BENCHMARK FOR BI-LEVEL TRUCK-DRONE OPTIMIZATION
%
% IMPROVEMENTS OVER CPU VERSION:
% - GPU-accelerated population-based algorithms (PSO, BOA, BSO)
% - Vectorized fitness evaluations on GPU
% - Parallel cost calculations using gpuArray
% - Optimized memory transfers between CPU and GPU
%
% REQUIREMENTS:
% - MATLAB with Parallel Computing Toolbox
% - CUDA-capable GPU (tested on RTX 3060Ti)
%
% Author: Heictor Costa
% Date:   February 06, 2026
% =========================================================================
clear; close all; clc;

benchmark_timer = tic;

%% SECTION 0: GPU AND PARALLEL SETUP
fprintf('Checking GPU availability...\n');
try
    gpu = gpuDevice(1);
    fprintf('GPU detected: %s (%.1f GB memory)\n', gpu.Name, gpu.AvailableMemory/1e9);
    reset(gpu);
cat ch ME
    warning('GPU not available: %s. Falling back to CPU.', ME.message);
end

cleanup_crash_dumps();

if isempty(gcp('nocreate'))
    numCores = feature('numcores');
    poolObj = parpool('local', numCores, 'SpmdEnabled', false);
    poolObj.IdleTimeout = Inf;
else
    disp('Parallel pool already running.');
end

%% SECTION 1: BENCHMARK CONFIGURATION
NUM_RUNS = 10;
BASE_OUTPUT_DIR = 'Benchmark_Results_GPU';
algorithms_to_test = {
    {'GA', 'BOA'},   {'GA', 'PSO'},   {'ACO', 'BSO'},...
    {'ALNS', 'PSO'}, {'ACO', 'CMAES'}
};

scenarios = struct();
scenarios(1).name = 'Scenario_1_Low_Complexity'; scenarios(1).numTargets = 5; scenarios(1).numObstacles = 25;
scenarios(2).name = 'Scenario_2_Medium_Complexity'; scenarios(2).numTargets = 8; scenarios(2).numObstacles = 40;
scenarios(3).name = 'Scenario_3_High_Complexity'; scenarios(3).numTargets = 10; scenarios(3).numObstacles = 60;

%% SECTION 2: CREATE JOB LIST
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

%% SECTION 3: PARALLEL EXECUTION WITH GPU
fprintf('\nStarting GPU-accelerated parallel execution...\n');
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
    sim_params.use_gpu = true;
    
    try
        [final_cost, comp_time] = runSingleSimulation_GPU(sim_params);
        results_data{i} = struct('scenario', scenario.name, 'algorithm', algo_name, 'cost', final_cost, 'time', comp_time);
        if isinf(final_cost)
            fprintf('Finished: [%s] - [%s] - Run %d | Cost: Inf (No feasible solution)\n', scenario.name, algo_name, run_idx);
        else
            fprintf('Finished: [%s] - [%s] - Run %d | Cost: %.2f\n', scenario.name, algo_name, run_idx, final_cost);
        end
    catch ME
        fprintf('FATAL ERROR in [%s] - [%s] - Run %d: %s\n', scenario.name, algo_name, run_idx, ME.message);
        results_data{i} = struct('scenario', scenario.name, 'algorithm', algo_name, 'cost', Inf, 'time', NaN);
    end
end
fprintf('\nAll parallel jobs complete.\n');

cleanup_crash_dumps();

%% SECTION 4: REPORT GENERATION (SAME AS CPU VERSION)
fprintf('Generating final report...\n');
if ~exist(BASE_OUTPUT_DIR, 'dir'), mkdir(BASE_OUTPUT_DIR); end
summary_file_path = fullfile(BASE_OUTPUT_DIR, 'benchmark_summary_gpu.csv');
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
        
        finite_costs = costs(isfinite(costs));
        if isempty(finite_costs)
            new_row = {s_name, a_name, Inf, NaN, Inf, Inf, mean(times, 'omitnan')};
        else
            new_row = {s_name, a_name, mean(finite_costs), std(finite_costs), min(finite_costs), max(finite_costs), mean(times, 'omitnan')};
        end
        results_table = [results_table; new_row];
    end
end
results_table.Properties.VariableNames = {'Scenario', 'Algorithm', 'Mean_Cost_USD', 'Std_Dev_Cost', 'Min_Cost_USD', 'Max_Cost_USD', 'Mean_Time_sec'};

disp('======================================================');
disp('          GPU-Accelerated Benchmark Summary           ');
disp('======================================================');
disp(results_table);

fprintf('Writing summary to: %s\n', summary_file_path);
writetable(results_table, summary_file_path);

total_benchmark_time_sec = toc(benchmark_timer);
fprintf('\n------------------------------------------------------\n');
fprintf('Total GPU Benchmark Time: %.2f minutes (%.2f hours).\n', total_benchmark_time_sec / 60, total_benchmark_time_sec / 3600);
fprintf('------------------------------------------------------\n');

cleanup_crash_dumps();
delete(gcp('nocreate'));

function cleanup_crash_dumps()
    try
        dump_pattern = fullfile(tempdir, 'matlab_crash_dump*');
        dump_files = dir(dump_pattern);
        if ~isempty(dump_files)
            for i = 1:length(dump_files)
                delete(fullfile(dump_files(i).folder, dump_files(i).name));
            end
            fprintf('Cleaned up %d crash dump file(s).\n', length(dump_files));
        end
        
        local_cluster = parcluster('local');
        if ~isempty(local_cluster.JobStorageLocation) && exist(local_cluster.JobStorageLocation, 'dir')
            job_files = dir(fullfile(local_cluster.JobStorageLocation, '**', '*'));
            old_files = job_files(~[job_files.isdir] & ([job_files.datenum] < now - 1));
            for i = 1:length(old_files)
                try
                    delete(fullfile(old_files(i).folder, old_files(i).name));
                catch
                end
            end
        end
    catch ME
        fprintf('Warning: Could not clean crash dumps: %s\n', ME.message);
    end
end