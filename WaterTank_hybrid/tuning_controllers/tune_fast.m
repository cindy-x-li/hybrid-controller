% =========================================================================
% COARSE PI-CONTROLLER GRID SEARCH
% Goal: Find Kp/Ki combos, track step response metrics
% =========================================================================
clear; clc;

%% 1. Plant Parameters & Targets
A = 100; b = 6.17; c = 4.39;
u_max = 4.5; u_min = 0.0;
ref = 10.0;
target_peak = 10.2; % Strict maximum boundary

% Adjusted precision settings for ode45
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);

%% 2. The Coarse Tuning Grid
Kp_values = 2.0 : 0.5 : 15.0;   
Ki_values = 0.5 : 0.1 : 5.0; 

%% 3. Setup Grid Search
Ts = 0.1;
T_final = 90; 
time_steps = 0:Ts:T_final;

total_runs = length(Kp_values) * length(Ki_values);
% Expanded pre-allocation: [Kp, Ki, Peak, Settle, RiseTime, PeakTime]
all_results = zeros(total_runs, 6); 
run_idx = 1;

fprintf('Starting Grid Search for %d combinations...\n', total_runs);
fprintf('Please wait...\n');
tic; % Start stopwatch

%% 4. The Search Loop
for i = 1:length(Kp_values)
    Kp = Kp_values(i);
    
    for j = 1:length(Ki_values)
        Ki = Ki_values(j);
        
        H_current = 0.0;
        integral_sum = 0.0;
        max_H = 0.0;
        abort_flag = false;
        
        % ecord the history for stepinfo to analyze
        H_history = zeros(1, length(time_steps));
        H_history(1) = H_current;
        
% Simulation Loop
        for k = 1:length(time_steps)-1
            error = ref - H_current;
            
            % --- ANTI-WINDUP CLAMPING ---
            u_test = (Kp * error) + (Ki * integral_sum);
            if u_test <= u_max && u_test >= u_min
                integral_sum = integral_sum + (error * Ts);
            end
            
            u_raw = (Kp * error) + (Ki * integral_sum);
            u_applied = max(u_min, min(u_max, u_raw));
            
            % ode45 Integration
            [~, H_out] = ode45(@(t, H) ((b/A)*u_applied) - ((c/A)*sqrt(abs(H))), [0, Ts], H_current, options);
            H_current = max(0, H_out(end));
            
            H_history(k+1) = H_current;
            
            % Track the peak
            if H_current > max_H
                max_H = H_current;
            end
            
            % EARLY ABORT: > 10.2
            if H_current > target_peak
                abort_flag = true;
                break;
            end
        end
        
        % --- Process the Results for this Kp/Ki pair ---
        if abort_flag
            % Aborted: log height at crash, Inf for all time metrics
            all_results(run_idx, :) = [Kp, Ki, max_H, Inf, Inf, Inf];
        else
            % Survived: let stepinfo analyze the full trajectory
            % 1% threshold creates a success band of +/- 0.1 cm (9.9 to 10.1)
            S = stepinfo(H_history, time_steps, ref, 'SettlingTimeThreshold', 0.01);
            
            max_height = S.Peak;
            settling_time = S.SettlingTime;
            rise_time = S.RiseTime;
            peak_time = S.PeakTime;
            
            % If it never stabilized permanently, mark settling as Inf
            if isnan(settling_time) || settling_time >= T_final
                settling_time = Inf;
            end
            
            % If it somehow never reached 90% of target, mark rise time as Inf
            if isnan(rise_time)
                rise_time = Inf;
            end
            
            all_results(run_idx, :) = [Kp, Ki, max_height, settling_time, rise_time, peak_time];
        end
        
        run_idx = run_idx + 1;
    end
end

search_time = toc; % Stop stopwatch

%% 5. Process and Export Results
fprintf('Grid Search completed in %.1f seconds.\n', search_time);

% Sort by Settling Time (Col 4). If tied, sort by Fastest Rise Time (Col 5)
all_results = sortrows(all_results, [4, 5]);

% --- EXCEL EXPORT ---
% Convert the sorted array into a Table with column headers
T_results = array2table(all_results, ...
    'VariableNames', {'Kp', 'Ki', 'Peak_cm', 'SettlingTime_s', 'RiseTime_s', 'PeakTime_s'});

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmm'));
fast_foldername = fullfile('2_tuning_controllers', 'best_fast');
if ~exist(fast_foldername, 'dir')
    mkdir(fast_foldername);
end

excel_filename = sprintf('fast_%s.xlsx', timestamp);
full_file_path = fullfile(fast_foldername, excel_filename);

%Write the file
writetable(T_results, full_file_path);
fprintf('Successfully saved all %d combinations to: %s\n', total_runs, full_file_path);

% --- FILTER FOR THE ABSOLUTE BEST RESULT ---
% Strict rules: Peak MUST be <= 10.2 AND Settling Time cannot be Inf
valid_mask = (all_results(:, 3) < target_peak) & (all_results(:, 4) < Inf);
valid_combinations = all_results(valid_mask, :);

fprintf('\n========================================\n');
fprintf('BEST CONTROLLER METRICS\n');
fprintf('========================================\n');

if isempty(valid_combinations)
    fprintf('STATUS                 : FAILED\n');
    fprintf('No combinations successfully kept the peak under 10.2 cm \n');
    fprintf('while successfully settling inside the 9.9 - 10.1 cm band.\n');
else
    % Because we already sorted, the first row is mathematically the best
    best_Kp = valid_combinations(1, 1);
    best_Ki = valid_combinations(1, 2);
    best_peak = valid_combinations(1, 3);
    best_settle = valid_combinations(1, 4);
    best_rise = valid_combinations(1, 5);
    best_peak_time = valid_combinations(1, 6);
    
    fprintf('Best Kp                : %.1f\n', best_Kp);
    fprintf('Best Ki                : %.2f\n', best_Ki);
    fprintf('----------------------------------------\n');
    fprintf('Maximum Height Reached : %.3f cm\n', best_peak);
    fprintf('True Settling Time     : %.2f seconds\n', best_settle);
    fprintf('Rise Time (10%% to 90%%) : %.2f seconds\n', best_rise);
    fprintf('Time to Hit Peak       : %.2f seconds\n', best_peak_time);
end
fprintf('========================================\n');