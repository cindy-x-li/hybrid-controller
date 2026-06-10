% =========================================================================
% PLOT & LOG PI-CONTROLLER
% save data to xlsx file
% =========================================================================
clear; clc; close all;

%% 1. Plant Parameters
A = 100;
b = 6.17;
c = 4.39;
u_max = 4.5;
u_min = 0.0;
ref = 10.0;

%% 2. PI Gains
best_Kp = 7;   
best_Ki = 12;    

%% 3. Simulation Setup
Ts = 0.1;
Tf = 80; 
time_steps = 0:Ts:Tf;
H_current = 0.0;
integral_sum = 0;

% Arrays to track all internal physics
H_history = zeros(1, length(time_steps));
u_history = zeros(1, length(time_steps));
error_history = zeros(1, length(time_steps));
p_term_history = zeros(1, length(time_steps)); % NEW: Track Proportional Voltage
integral_history = zeros(1, length(time_steps));
u_raw_history = zeros(1, length(time_steps));
dHdt_history = zeros(1, length(time_steps));

%% 4. Simulation Loop (Standalone PI with Anti-Windup Clamping)
for k = 1:length(time_steps)-1
    t_current = time_steps(k);
    t_next = time_steps(k+1);
    
    H_history(k) = H_current;
    error = ref - H_current;
    error_history(k) = error; 
    
    % NEW: Log the Proportional Term (Kp * error)
    p_term = best_Kp * error;
    p_term_history(k) = p_term;
    
    % Anti-Windup Clamping (beyond 4.5 Volts)
    u_test = p_term + (best_Ki * integral_sum);
    if u_test <= u_max && u_test >= u_min
        integral_sum = integral_sum + (error * Ts); 
    end
    integral_history(k) = integral_sum; 
    
    u_raw = p_term + (best_Ki * integral_sum);
    u_raw_history(k) = u_raw; 
    
    u_applied = max(u_min, min(u_max, u_raw));
    u_history(k) = u_applied; 
    
    dHdt_history(k) = water_tank_dynamics(t_current, H_current, u_applied, A, b, c);
    
    [~, H_out] = ode45(@(t, H) water_tank_dynamics(t, H, u_applied, A, b, c), [t_current, t_next], H_current);
    H_current = max(0, H_out(end)); 
end

% Pad the final elements
H_history(end) = H_current;
u_history(end) = u_history(end-1);
error_history(end) = error_history(end-1);
p_term_history(end) = p_term_history(end-1); 
integral_history(end) = integral_history(end-1);
u_raw_history(end) = u_raw_history(end-1);
dHdt_history(end) = 0; 

%% --- Export Physics Data to XLSX ---
% Added p_term_history between error and integral
PhysicsData = table(time_steps', H_history', error_history', p_term_history', integral_history', ...
    u_raw_history', u_history', dHdt_history', ...
    'VariableNames', {'Time_s', 'Height_cm', 'Error_cm', 'P_Term_V', 'IntegralSum', 'U_Raw_V', 'U_Applied_V', 'dHdt'});

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmm'));
fast_foldername = fullfile('tuning_controllers', 'system_tracking');
if ~exist(fast_foldername, 'dir')
    mkdir(fast_foldername);
end
excel_filename = sprintf('log_%s.xlsx', timestamp);
full_file_path = fullfile(fast_foldername, excel_filename);

writetable(PhysicsData, full_file_path);
fprintf('========================================\n');
fprintf('Results successfully saved to:\n%s\n', full_file_path);

%% 5. Calculate Metrics
S = stepinfo(H_history, time_steps, ref, SettlingTimeThreshold=0.01);
fprintf('========================================\n');
fprintf('CONTROLLER METRICS (Kp = %.1f, Ki = %.2f)\n', best_Kp, best_Ki);
fprintf('========================================\n');
fprintf('Maximum Height Reached : %.3f cm\n', S.Peak);
fprintf('Rise Time (10%% to 90%%) : %.2f seconds\n', S.RiseTime);
if isnan(S.SettlingTime) || S.SettlingTime >= Tf
    fprintf('True Settling Time     : FAILED TO SETTLE\n');
else
    fprintf('True Settling Time     : %.2f seconds\n', S.SettlingTime);
end

%% 6. Key Physics Milestones
fprintf('========================================\n');
fprintf('KEY PHYSICS MILESTONES\n');
fprintf('========================================\n');

wakeup_idx = find(u_raw_history < 4.5, 1, 'first');
if ~isempty(wakeup_idx)
    fprintf('INTEGRATOR WAKE-UP:\n');
    fprintf('  Time       : %.1f s\n', time_steps(wakeup_idx));
    fprintf('  Height     : %.3f cm\n', H_history(wakeup_idx));
    fprintf('  P_Term     : %.3f V\n', p_term_history(wakeup_idx));
    fprintf('  U_Raw      : %.3f V\n', u_raw_history(wakeup_idx));
end

[~, peak_idx] = max(H_history);
fprintf('\nEXACT PEAK (Highest Bounce):\n');
fprintf('  Time       : %.1f s\n', time_steps(peak_idx));
fprintf('  Height     : %.3f cm\n', H_history(peak_idx));
fprintf('  P_Term     : %.3f V (Braking Force)\n', p_term_history(peak_idx));
fprintf('  Integral   : %.3f\n', integral_history(peak_idx));
fprintf('  dH/dt      : %.3f\n', dHdt_history(peak_idx));
fprintf('========================================\n');

%% 7. Plotting
figure('Name', 'Optimal PI-Controller', 'Position', [100, 100, 800, 600], 'Color', 'w');

subplot(2,1,1);
plot(time_steps, H_history, 'b', 'LineWidth', 2); hold on;
yline(ref, 'k-', 'Target (10.0 cm)', 'LineWidth', 1.5);
yline(10.2, 'r:', 'Max Overshoot (10.2 cm)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(10.1, 'g--', 'Upper Bound (10.1 cm)', 'LineWidth', 1.0);
yline(9.9, 'g--', 'Lower Bound (9.9 cm)', 'LineWidth', 1.0);
title(sprintf('Water Level (Kp = %.2f, Ki = %.2f) - Peak: %.3f cm', best_Kp, best_Ki, S.Peak));
ylabel('Height (cm)'); grid on;
xlabel('Time (s)');
ylim([0, 12]);
xlim([0, Tf]);

subplot(2,1,2);
plot(time_steps, u_history, 'y', 'LineWidth', 1.5); hold on;
plot(time_steps, u_raw_history, 'c:', 'LineWidth', 1.0); 
yline(u_max, 'r:', 'Max Voltage (4.5V)', 'LabelHorizontalAlignment', 'left');
yline(u_min, 'r:', 'Min Voltage (0V)', 'LabelHorizontalAlignment', 'left');
legend('Applied Voltage', 'Raw Voltage (Pre-Clamp)', 'Location', 'best');
title('Pump Voltage (u)');
xlabel('Time (s)'); ylabel('Volts (V)'); 
ylim([-1.0, 6]); xlim([0, Tf]);
grid on;

%% Physics Engine
function dHdt = water_tank_dynamics(~, H, u, A, b, c)
    if H < 0, outflow = 0; else, outflow = (c / A) * sqrt(H); end
    inflow = (b / A) * u;
    dHdt = inflow - outflow;
end