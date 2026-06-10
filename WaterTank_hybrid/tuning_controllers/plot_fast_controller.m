% =========================================================================
% PLOT FAST PI-CONTROLLER (With Metrics)
% includes a zoomed in view
% included in presentation
% =========================================================================
clear; clc; close all;

%% 1. Plant Parameters
A = 100;
b = 6.17;
c = 4.39;
u_max = 4.5;
u_min = 0.0;
ref = 10.0;

%% 2. PI Gains (Plug in your Grid Search winners here!)
best_Kp = 7;   
best_Ki = 12;    

%% 3. Simulation Setup
Ts = 0.1;
Tf = 80; % Give it plenty of time to show the settling phase
time_steps = 0:Ts:Tf;

H_current = 0.0;
integral_sum = 0;

H_history = zeros(1, length(time_steps));
u_history = zeros(1, length(time_steps));

%% 4. Simulation Loop (Standalone PI with Anti-Windup Clamping)
for k = 1:length(time_steps)-1
    t_current = time_steps(k);
    t_next = time_steps(k+1);
    
    H_history(k) = H_current;
    error = ref - H_current;
    
    % Anti-Windup Clamping (beyond 4.5 Volts)
    u_test = (best_Kp * error) + (best_Ki * integral_sum);
    if u_test <= u_max && u_test >= u_min
        integral_sum = integral_sum + (error * Ts); 
    end
    
    u_raw = (best_Kp * error) + (best_Ki * integral_sum);

    u_applied = max(u_min, min(u_max, u_raw));
    u_history(k) = u_applied;
    
    % Physics Engine
    [~, H_out] = ode45(@(t, H) water_tank_dynamics(t, H, u_applied, A, b, c), [t_current, t_next], H_current);
    H_current = max(0, H_out(end)); 
end
H_history(end) = H_current;
u_history(end) = u_history(end-1);

% CALCULATE FINAL METRICS AT T_FINAL
H_final = H_history(end);
steady_state_error = abs(ref - H_final);

%% 5. Calculate Metrics (Using our 1% success band)
S = stepinfo(H_history, time_steps, ref, SettlingTimeThreshold=0.01);

fprintf('\n========================================\n');
fprintf('CONTROLLER METRICS (Kp = %.1f, Ki = %.2f)\n', best_Kp, best_Ki);
fprintf('========================================\n');
fprintf('Maximum Height Reached : %.3f cm\n', S.Peak);
fprintf('Maximum Height Time : %.2f sec\n', S.PeakTime);
fprintf('Rise Time (10%% to 90%%) : %.2f seconds\n', S.RiseTime);

if isnan(S.SettlingTime) || S.SettlingTime >= Tf
    fprintf('Settling Time     : FAILED TO SETTLE\n');
else
    fprintf('Settling Time     : %.2f seconds\n', S.SettlingTime);

fprintf('Steady-State Error     : %.2e cm\n', steady_state_error);
end
fprintf('========================================\n');

% %% 6. Plotting
% figure('Name', 'Optimal PI-Controller', 'Position', [100, 100, 800, 600], 'Color', 'w');
% 
% subplot(2,1,1);
% plot(time_steps, H_history, 'b', 'LineWidth', 2); hold on;
% yline(ref, 'k-', 'Target (10.0 cm)', 'LineWidth', 1.5);
% yline(10.2, 'r:', 'Max Overshoot (10.2 cm)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
% yline(10.1, 'g--', 'Upper Bound (10.1 cm)', 'LineWidth', 1.0);
% yline(9.9, 'g--', 'Lower Bound (9.9 cm)', 'LineWidth', 1.0);
% 
% title(sprintf('Water Level (Kp = %.2f, Ki = %.2f) - Peak: %.3f cm', best_Kp, best_Ki, S.Peak));
% ylabel('Height (cm)'); grid on;
% xlabel('Time (s)');
% ylim([0, 12]);
% xlim([0, Tf]);
% 
% subplot(2,1,2);
% plot(time_steps, u_history, 'y', 'LineWidth', 1.5); hold on;
% yline(u_max, 'r:', 'Max Voltage (4.5V)', 'LabelHorizontalAlignment', 'left');
% yline(u_min, 'r:', 'Min Voltage (0V)', 'LabelHorizontalAlignment', 'left');
% 
% title('Pump Voltage (u)');
% xlabel('Time (s)'); ylabel('Volts (V)'); 
% ylim([-0.5, 5]); xlim([0, Tf]);
% grid on;

%% 6. Plotting Results
% Match the figure size and white background of the hybrid plot
figure('Name', 'Optimal PI-Controller', 'Position', [100, 100, 850, 750], 'Color', 'w');

% --- SUBPLOT 1: WATER HEIGHT ---
subplot(2,1,1); hold on;

% Use the hex colors from your preferred hybrid style
% yline(10.0, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Target (10.0 cm)');
yline(10.2, ':', 'Color', '#FF8C00', 'LineWidth', 1.5, 'DisplayName', 'Max Overshoot (2%)');
yline(10.1, '--', 'Color', '#006400', 'LineWidth', 1.0, 'DisplayName', 'Tolerance Band (\pm 1%)');
yline(9.9, '--', 'Color', '#006400', 'LineWidth', 1.0, 'HandleVisibility', 'off');

% Plot the single controller's height history as a solid line
plot(time_steps, H_history, 'r', 'LineWidth', 1.5, 'DisplayName', 'PI Controller');

ylabel('Height (cm)'); 
xlabel('Time (s)'); 
grid on; ylim([0, 11]); xlim([0, Tf]);

% Match legend placement to the hybrid plot
lgd = legend('Location', 'northwest'); 
lgd.Position(2) = lgd.Position(2) - 0.07;
% title(sprintf('Water Level (Kp = %.2f, Ki = %.2f) - Peak: %.3f cm', best_Kp, best_Ki, S.Peak));

% ==========================================
% INSET ZOOM PLOT (Shifted to Bottom Right)
% ==========================================
% left, bottom, width, height
axes('Position', [0.67, 0.65, 0.20, 0.18]); 
hold on; box on;

% yline(10.0, 'k-', 'LineWidth', 1.5);
yline(10.2, ':', 'Color', '#FF8C00', 'LineWidth', 1.5);
yline(10.1, '--', 'Color', '#006400', 'LineWidth', 1.0);
yline(9.9, '--', 'Color', '#006400', 'LineWidth', 1.0);

% Plot the same line in the inset
plot(time_steps, H_history, 'r', 'LineWidth', 1.5);

% You may need to adjust xlim based on where this specific controller settles
xlim([53, 70]); 
ylim([9.85, 10.25]); 
grid on;
title('Transient to Steady-State', 'FontSize', 9);

% ==========================================
% --- SUBPLOT 2: APPLIED VOLTAGE ---
subplot(2,1,2); hold on;

% Plotting as blue instead of yellow for high contrast
plot(time_steps, u_history, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Pump Voltage');

% Match the Max/Min limits to the hybrid plot style
yline(u_max, ':', 'Color', '#FF8C00', 'LineWidth', 1.5, 'DisplayName', 'Max Pump Limit (4.5V)');
yline(u_min, ':', 'Color', '#FF8C00', 'LineWidth', 1.5, 'DisplayName', 'Min Pump Limit (0V)');

%title('Pump Voltage (u)');
xlabel('Time (s)'); 
ylabel('Volts (V)'); 
ylim([-0.5, 5]); xlim([0, Tf]);
grid on;

lgd = legend('Location', 'southwest');
lgd.Position(2) = lgd.Position(2) + 0.04;

%% Physics Engine
function dHdt = water_tank_dynamics(~, H, u, A, b, c)
    if H < 0, outflow = 0; else, outflow = (c / A) * sqrt(H); end
    inflow = (b / A) * u;
    dHdt = inflow - outflow;
end