% =========================================================
% PLOT & LOG 1-STEP LOOKAHEAD SUPERVISOR
% =========================================================
clear; clc; close all;

%% 1. Plant Parameters
A = 100; b = 6.17; c = 4.39;
u_max = 4.5; u_min = 0.0; 
ref = 10.0;

%% 2. Controller Gains
Kpf = 7;   Kif = 12;    % Fast Expert
Kps = 25;  Kis = 11;    % Slow Expert

%% 3. Simulation Setup
Ts = 0.1;
Tf = 80; 
time_steps = 0:Ts:Tf;
H_current = 0.0;
shared_integral = 0; % Shared to prevent voltage spikes

% --- PREALLOCATION ---
H_history = zeros(1, length(time_steps));
u_history = zeros(1, length(time_steps));
active_controller = zeros(1, length(time_steps));
error_history = zeros(1, length(time_steps));

% New arrays for deep tracking
u_f_history = zeros(1, length(time_steps));
u_s_history = zeros(1, length(time_steps));
error_f_history = zeros(1, length(time_steps));
error_s_history = zeros(1, length(time_steps));

options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);

%% 4. Supervisory Rules Setup
% Calculate epsilon based on the 1% Settling Rule
settling_bound = 0.01 * ref;            % 1% of target (0.1 cm)
safety_multiplier = 2;                  % Safety buffer multiplier
epsilon = settling_bound * safety_multiplier; % error boundary
fprintf('Running 1-Step Lookahead (Epsilon = %.2f cm)...\n', epsilon);

%% 5. Master Control Loop
for k = 1:length(time_steps)-1
    t_current = time_steps(k);
    t_next = time_steps(k+1);
    
    H_history(k) = H_current;
    error = ref - H_current;
    error_history(k) = error;
    
    % Track previous controller for the stickiness rule
    if k == 1
        prev_ctrl = 1; % Default to Fast at the start
    else
        prev_ctrl = active_controller(k-1);
    end
    
    % --- STEP 1: 1-STEP EULER PREDICTION ---
    U_f_raw = (Kpf * error) + (Kif * shared_integral);
    U_s_raw = (Kps * error) + (Kis * shared_integral);
    
    U_f_test = max(u_min, min(u_max, U_f_raw));
    U_s_test = max(u_min, min(u_max, U_s_raw));
    
    outflow = (c / A) * sqrt(max(0, H_current));
    H_pred_f = H_current + (((b / A) * U_f_test - outflow) * Ts);
    H_pred_s = H_current + (((b / A) * U_s_test - outflow) * Ts);
    
    Error_f = abs(ref - H_pred_f);
    Error_s = abs(ref - H_pred_s);
    
    % Log the independent controller metrics
    u_f_history(k) = U_f_test;
    u_s_history(k) = U_s_test;
    error_f_history(k) = Error_f;
    error_s_history(k) = Error_s;
    
    % --- STEP 2: SUPERVISORY LOGIC ---
    if Error_f <= epsilon && Error_s <= epsilon
        % RULE 1: stay with the prev controller
        if prev_ctrl == 1
            Kp_active = Kpf; 
            Ki_active = Kif; 
            active_controller(k) = 1;
        else
            Kp_active = Kps; 
            Ki_active = Kis; 
            active_controller(k) = 2;
        end
        
    else
        % RULE 2: use controller with smaller error
        if Error_f <= Error_s 
            Kp_active = Kpf; 
            Ki_active = Kif; 
            active_controller(k) = 1;
        else
            Kp_active = Kps; 
            Ki_active = Kis; 
            active_controller(k) = 2;
        end
    end
    
    % --- STEP 3: APPLY THE CHOSEN CONTROL ---
    u_test = (Kp_active * error) + (Ki_active * shared_integral);
    
    % Anti-Windup Conditional Integration
    if u_test <= u_max && u_test >= u_min
        shared_integral = shared_integral + (error * Ts);
    end
    
    u_raw = (Kp_active * error) + (Ki_active * shared_integral);
    u_applied = max(u_min, min(u_max, u_raw));
    u_history(k) = u_applied;
    
    % Real Physics Step
    [~, H_out] = ode45(@(t, H) water_tank_dynamics(t, H, u_applied, A, b, c), [t_current, t_next], H_current, options);
    H_current = max(0, H_out(end));
end

% Finalize the last index of the preallocated arrays
H_history(end) = H_current;
error_history(end) = ref - H_current;
u_history(end) = u_history(end-1);
active_controller(end) = active_controller(end-1);
u_f_history(end) = u_f_history(end-1);
u_s_history(end) = u_s_history(end-1);
error_f_history(end) = error_f_history(end-1);
error_s_history(end) = error_s_history(end-1);

%% 6. Data Logging to Excel
% Create a structured table of the results
log_table = table(time_steps', H_history', error_history', u_history', active_controller', ...
                  u_f_history', u_s_history', error_f_history', error_s_history', ...
                  'VariableNames', {'Time_s', 'Height_cm', 'Error_cm', 'U_Applied_V', ...
                                    'Active_Expert', 'U_Applied_Fast', 'U_Applied_Slow', ...
                                    'Error_Fast', 'Error_Slow'});

% Define directory and verify it exists
save_dir = fullfile('WaterTank_hybrid', '2_tuning_controllers', 'system_tracking');
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% Generate filename with timestamp and save
file_name = sprintf('hybrid_log_%s.xlsx', datestr(now, 'yyyymmdd_HHMM'));
full_path = fullfile(save_dir, file_name);
writetable(log_table, full_path);

fprintf('System tracking data saved successfully to:\n%s\n', full_path);

%% 7. Plotting Results
figure('Name', 'Rule-Based 1-Step Hybrid', 'Position', [100, 100, 850, 750], 'Color', 'w');
% --- SUBPLOT 1: WATER HEIGHT ---
subplot(2,1,1); hold on;
% Added DisplayNames back so they populate the top legend
yline(10.2, ':', 'Color', '#FF8C00', 'LineWidth', 1.5, 'DisplayName', 'Max Overshoot (2%)');
yline(10.1, '--', 'Color', '#006400', 'LineWidth', 1.0, 'DisplayName', 'Tolerance Band (\pm 1%)');
yline(9.9, '--', 'Color', '#006400', 'LineWidth', 1.0, 'HandleVisibility', 'off');
fast_idx = find(active_controller == 1);
slow_idx = find(active_controller == 2);
% Ensured scatter plots have DisplayNames for the top legend
scatter(time_steps(fast_idx), H_history(fast_idx), 15, 'r', 'filled', 'DisplayName', 'Fast Expert');
scatter(time_steps(slow_idx), H_history(slow_idx), 15, 'b', 'filled', 'DisplayName', 'Slow Expert');
plot(time_steps, H_history, 'k:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
ylabel('Height (cm)'); 
xlabel('Time (s)'); 
grid on; ylim([0, 11]);
% Placed the legend in the top-left to avoid the data line and the inset plot!
lgd = legend('Location', 'northwest'); 
lgd.Position(2) = lgd.Position(2) - 0.05;

% ==========================================
% --- SUBPLOT 2: APPLIED VOLTAGE ---
subplot(2,1,2); hold on;
plot(time_steps, u_history, 'w:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
scatter(time_steps(fast_idx), u_history(fast_idx), 10, 'r', 'filled', 'DisplayName', 'Fast Expert');
scatter(time_steps(slow_idx), u_history(slow_idx), 10, 'b', 'filled', 'DisplayName', 'Slow Expert');
yline(u_max, ':', 'Color', '#FF8C00', 'LineWidth', 1.5, 'DisplayName', 'Max Pump Limit (4.5V)');
yline(u_min, ':', 'Color', '#FF8C00', 'LineWidth', 1.5, 'DisplayName', 'Min Pump Limit (0V)');
ylabel('Volts (V)'); grid on; ylim([-0.5, 5]);
xlabel('Time (s)'); 
lgd1 = legend('Location', 'southwest');
lgd1.Position(2) = lgd1.Position(2) + 0.05;

%% 8. Final Evaluation Metrics
S_hybrid = stepinfo(H_history, time_steps, ref, 'SettlingTimeThreshold', 0.01);
fprintf('\n========================================\n');
fprintf('Results (Epsilon = %.2f)\n', epsilon);
fprintf('========================================\n');
fprintf('Peak Height        : %.3f cm\n', S_hybrid.Peak);
fprintf('Peak Height Time   : %.2f sec\n', S_hybrid.PeakTime);
fprintf('Rise Time (10-90%%) : %.2f seconds\n', S_hybrid.RiseTime);
if isnan(S_hybrid.SettlingTime) || S_hybrid.SettlingTime >= Tf
    fprintf('True Settling Time : FAILED TO SETTLE\n');
else
    fprintf('True Settling Time : %.2f seconds\n', S_hybrid.SettlingTime);
end
steady_state_error = abs(ref - H_history(end));
fprintf('Steady-State Error : %.2e cm\n', steady_state_error);
fprintf('========================================\n');

%% Physics Engine Function
function dHdt = water_tank_dynamics(~, H, u, A, b, c)
    if H < 0, outflow = 0; else, outflow = (c / A) * sqrt(H); end
    inflow = (b / A) * u;
    dHdt = inflow - outflow;
end