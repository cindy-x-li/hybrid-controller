% =========================================================================
% PLOT NOT CLAMPED PI-CONTROLLER
% demonstrates overshoot due to integral gain
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
best_Kp = 3;   
best_Ki = 0.2;    

%% 3. Simulation Setup
Ts = 0.1;
T_final = 180; 
time_steps = 0:Ts:T_final;

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
    
    % NO CLAMPING!! see the huge overshoot
    u_test = (best_Kp * error) + (best_Ki * integral_sum);

    integral_sum = integral_sum + (error * Ts); 
    
    u_raw = (best_Kp * error) + (best_Ki * integral_sum);

    u_applied = max(u_min, min(u_max, u_raw));
    u_history(k) = u_applied;
    
    % Physics Engine
    [~, H_out] = ode45(@(t, H) water_tank_dynamics(t, H, u_applied, A, b, c), [t_current, t_next], H_current);
    H_current = max(0, H_out(end)); 
end
H_history(end) = H_current;
u_history(end) = u_history(end-1);

%% 5. Calculate Metrics (Using our 1% success band)
S = stepinfo(H_history, time_steps, ref, SettlingTimeThreshold=0.01);

fprintf('\n========================================\n');
fprintf('CONTROLLER METRICS (Kp = %.1f, Ki = %.2f)\n', best_Kp, best_Ki);
fprintf('========================================\n');
fprintf('Maximum Height Reached : %.3f cm\n', S.Peak);
fprintf('Rise Time (10%% to 90%%) : %.2f seconds\n', S.RiseTime);

if isnan(S.SettlingTime) || S.SettlingTime >= T_final
    fprintf('True Settling Time     : FAILED TO SETTLE\n');
else
    fprintf('True Settling Time     : %.2f seconds\n', S.SettlingTime);
end
fprintf('========================================\n');

%% 6. Plotting
figure('Name', 'Optimal PI-Controller', 'Position', [100, 100, 800, 600], 'Color', 'w');

subplot(2,1,1);
plot(time_steps, H_history, 'b', 'LineWidth', 2); hold on;
yline(ref, 'k-', 'Target (10.0 cm)', 'LineWidth', 1.5);
yline(10.2, 'r:', 'Max Overshoot (10.2 cm)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(10.1, 'g--', 'LineWidth', 1.0);
yline(9.9, 'g--', 'LineWidth', 1.0);

title(sprintf('Water Level (Kp = %.2f, Ki = %.2f) - Peak: %.3f cm', best_Kp, best_Ki, S.Peak));
ylabel('Height (cm)'); grid on;
xlabel('Time (s)');
ylim([0, 16]);
xlim([0, T_final]);

subplot(2,1,2);
plot(time_steps, u_history, 'y', 'LineWidth', 1.5); hold on;
yline(u_max, 'r:', 'Max Voltage (4.5V)', 'LabelHorizontalAlignment', 'left');
yline(u_min, 'r:', 'Min Voltage (0V)', 'LabelHorizontalAlignment', 'left');

title('Pump Voltage (u)');
xlabel('Time (s)'); ylabel('Volts (V)'); 
ylim([-0.5, 5]); xlim([0, T_final]);
grid on;

%% Physics Engine
function dHdt = water_tank_dynamics(~, H, u, A, b, c)
    if H < 0, outflow = 0; else, outflow = (c / A) * sqrt(H); end
    inflow = (b / A) * u;
    dHdt = inflow - outflow;
end