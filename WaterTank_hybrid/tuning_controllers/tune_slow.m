% =========================================================================
% SLOW PI GRID SEARCH (Fixed Horizon & Streamlined Metrics)
% Goal: Find Kp/Ki combos, track step response metrics
% =========================================================================
clear; clc;

%% 1. Plant Parameters & Targets
A = 100; b = 6.17; c = 4.39;
u_max = 4.5; u_min = 0.0;
ref = 10.0;

% STRICT ZERO-OVERSHOOT RULE
max_peak = 10.005; 

options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);

%% 2. The Precision Grid
Kp_values = 15.0 : 1.0 : 40.0;   
Ki_values = 3.0 : 0.5 : 11.0; 

%% 3. Setup Grid Search
Ts = 0.1;
T_final = 80; % Replace this with your calculated deadline!
time_steps = 0:Ts:T_final;

total_runs = length(Kp_values) * length(Ki_values);

% ALLOCATE 6 COLUMNS OF DATA
all_results = zeros(total_runs, 6); 
run_idx = 1;

fprintf('Starting ode45 Grid Search for %d precise combinations...\n', total_runs);
tic;

%% 4. The Standard Simulation Loop
for i = 1:length(Kp_values)
    Kp = Kp_values(i);
    for j = 1:length(Ki_values)
        Ki = Ki_values(j);
        
        H_current = 0.0;
        integral_sum = 0.0;
        max_H = 0.0;
        abort_flag = false;
        
        H_history = zeros(1, length(time_steps));
        H_history(1) = H_current;
        
        for k = 1:length(time_steps)-1
            error = ref - H_current;
            
            % STANDARD ANTI-WINDUP CLAMPING
            u_test = (Kp * error) + (Ki * integral_sum);
            if u_test <= u_max && u_test >= u_min
                integral_sum = integral_sum + (error * Ts);
            end
            
            u_raw = (Kp * error) + (Ki * integral_sum);
            u_applied = max(u_min, min(u_max, u_raw));
            
            [~, H_out] = ode45(@(t, H) ((b/A)*u_applied) - ((c/A)*sqrt(abs(H))), [0, Ts], H_current, options);
            H_current = max(0, H_out(end));
            
            H_history(k+1) = H_current;
            
            if H_current > max_H
                max_H = H_current;
            end
            
            % EARLY ABORT: > 10.005 cm
            if H_current > max_peak
                abort_flag = true;
                break;
            end
        end
        
        if abort_flag
            % Fill the 6 columns for an aborted run
            all_results(run_idx, :) = [Kp, Ki, max_H, Inf, Inf, Inf];
        else
            S = stepinfo(H_history, time_steps, ref, 'SettlingTimeThreshold', 0.01);
            
            % CALCULATE FINAL METRICS AT T_FINAL
            H_final = H_history(end);
            steady_state_error = abs(ref - H_final);
            
            % SAVE EXACTLY 6 COLUMNS: [Kp, Ki, Peak, PeakTime, FinalHeight, Error]
            all_results(run_idx, :) = [Kp, Ki, S.Peak, S.PeakTime, H_final, steady_state_error];
        end
        run_idx = run_idx + 1;
    end
end

search_time = toc;

%% 5. Process and Export Results
fprintf('Grid Search completed in %.1f seconds.\n', search_time);

% SORT BY COLUMN 6 (Steady State Error) to find the most accurate controller
all_results = sortrows(all_results, 6);

% Update Table Variable Names to match the 6 columns
T_results = array2table(all_results, ...
    'VariableNames', {'Kp', 'Ki', 'Peak_cm', 'PeakTime_s', 'FinalHeight_cm', 'SteadyStateError_cm'});

slow_foldername = fullfile('2_tuning_controllers', 'best_slow');
if ~exist(slow_foldername, 'dir')
    mkdir(slow_foldername);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmm'));
excel_filename = sprintf('slow_%s.xlsx', timestamp);
full_file_path = fullfile(slow_foldername, excel_filename);

writetable(T_results, full_file_path);
fprintf('Successfully saved data to: %s\n', full_file_path);

% --- FILTER FOR THE ABSOLUTE BEST SLOW CONTROLLER ---
% 1: STRICTLY OVERDAMPED
% The Peak cannot be meaningfully larger than the Final Height. 
% This mathematically guarantees the water never "bounced" or fell.
is_overdamped = (all_results(:, 3) - all_results(:, 5)) <= 1e-4;

% 2: PRECISE DOCKING
% Must achieve a Steady-State Error of 0.005 cm or less by T_final
is_precise = all_results(:, 6) <= 0.005;

% Apply both filter
valid_filter = is_overdamped & is_precise;
valid_combinations = all_results(valid_filter, :);

fprintf('\n========================================\n');
fprintf('BEST SLOW CONTROLLER (STRICTLY OVERDAMPED)\n');
fprintf('with smallest steady state error from reference height\n');
fprintf('========================================\n');

if isempty(valid_combinations)
    fprintf('STATUS                 : FAILED\n');
    fprintf('Could not find a strictly overdamped, precise controller.\n');
else
    % Sort strictly by Steady State Error (Column 6)
    valid_combinations = sortrows(valid_combinations, 6);
    
    fprintf('Best Kp                : %.1f\n', valid_combinations(1, 1));
    fprintf('Best Ki                : %.3f\n', valid_combinations(1, 2));
    fprintf('----------------------------------------\n');
    fprintf('Peak Height Reached    : %.4f cm\n', valid_combinations(1, 3));
    fprintf('Final Height at T=%d s : %.4f cm\n', T_final, valid_combinations(1, 5));
    fprintf('Steady-State Error     : %.6f cm\n', valid_combinations(1, 6));
end
fprintf('========================================\n');