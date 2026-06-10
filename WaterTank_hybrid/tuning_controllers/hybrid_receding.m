% =========================================================================
% RECEDING HORIZON PREDICTIVE SUPERVISOR 
% freezes the controller for 1 second to prevent chattering
% not used in presentation due to receding horizon exclusively look at fast
% branch instead of exploring BOTH fast and slow's control signals
% =========================================================================
clear; clc; close all;

%% 1. Plant Parameters
A = 100; b = 6.17; c = 4.39;
u_max = 4.5; u_min = 0.0; ref = 10.0;

%% 2. Controller Gains
Kpf = 7;   Kif = 12;    % Fast Expert
Kps = 25;  Kis = 11;    % Slow Expert

%% 3. Simulation Setup
Ts = 0.1;
Tf = 80; 
time_steps = 0:Ts:Tf;

% initial water height
H_current = 0.0;
shared_integral = 0; 

H_history = zeros(1, length(time_steps));
u_history = zeros(1, length(time_steps));
active_controller = zeros(1, length(time_steps));

% Horizon is 2.5 seconds into the future
T_horizon = 2.5; 
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);

% =========================================================
% HARDWARE PROTECTION: Asymmetric Dwell Time Setup
% =========================================================
slow_lock_timer = 0.0;     % Timer to track cooldown
lockout_duration = 1.0;    % Minimum time (seconds) to hold the Slow controller

fprintf('Running Hybrid Controller...\n');

%% 4. Control Loop
for k = 1:length(time_steps)-1
    t_current = time_steps(k);
    t_next = time_steps(k+1);
    
    H_history(k) = H_current;
    error = ref - H_current;
    
    % --- STEP 1: DECREMENT HARDWARE TIMER ---
    if slow_lock_timer > 0
        slow_lock_timer = slow_lock_timer - Ts;
    end
    
    % --- STEP 2: DYNAMIC SAFETY CHECK ---
    % 1 = safe, 0 = unsafe
    is_fast_safe = check_fast_safety_ode(H_current, shared_integral, Kpf, Kif, ref, u_max, u_min, A, b, c, T_horizon);
    
% --- STEP 3: ASYMMETRIC SELECTION LOGIC WITH DOCKING ZONE ---
    in_docking_zone = abs(error) <= 0.1;
    
    if in_docking_zone
        % 1. DOCKING: Force the precision at specific distance
        Kp_active = Kps;
        Ki_active = Kis;
        active_controller(k) = 2;
        
    elseif ~is_fast_safe
        % 2. DANGER: Fast is unsafe. Instantly brake with Slow and start hardware lock.
        Kp_active = Kps;
        Ki_active = Kis;
        active_controller(k) = 2;
        slow_lock_timer = lockout_duration; 
        
    elseif slow_lock_timer <= 0
        % 3. TRANSIT: Safe, not docking, and cooldown is over. Use Fast.
        Kp_active = Kpf;
        Ki_active = Kif;
        active_controller(k) = 1;
        
    else
        % 4. COOLDOWN: Physics are safe, but protecting pump from chatter.
        Kp_active = Kps;
        Ki_active = Kis;
        active_controller(k) = 2;
    end
    
    % --- STEP 4: APPLY THE CHOSEN CONTROL ---
    u_test = (Kp_active * error) + (Ki_active * shared_integral);
    if u_test <= u_max && u_test >= u_min
        shared_integral = shared_integral + (error * Ts);
    end
    
    u_raw = (Kp_active * error) + (Ki_active * shared_integral);
    u_applied = max(u_min, min(u_max, u_raw));
    u_history(k) = u_applied;
    
    % Real Physics Step
    [~, H_out] = ode45(@(t, H) water_tank_dynamics(t, H, u_applied, A, b, c), [t_current, t_next], H_current, options);
    H_current = max(0, H_out(end));
    
    % ========================================================
    % THE PERTURBATION TEST (Drop a boulder at t=60s)
    % ========================================================
    if abs(t_current - 60.0) < Ts/2
        H_current = 10.8; 
        error = ref - H_current;
        fprintf('PERTURBATION: Water spiked to 10.8cm at t=60s!\n');
    end
end

H_history(end) = H_current;
u_history(end) = u_history(end-1);
active_controller(end) = active_controller(end-1);

%% 5. Plotting Results
figure('Name', 'ode45 Predictive Hybrid', 'Position', [100, 100, 850, 750], 'Color', 'w');

% --- SUBPLOT 1: WATER HEIGHT ---
subplot(3,1,1); hold on;
yline(ref, 'y-', 'Target (10.0 cm)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(10.2, 'r:', 'Max Peak Safety Boundary', 'LineWidth', 1.5, 'HandleVisibility', 'off');
fast_idx = find(active_controller == 1);
slow_idx = find(active_controller == 2);
scatter(time_steps(fast_idx), H_history(fast_idx), 15, 'r', 'filled', 'DisplayName', 'Fast Expert');
scatter(time_steps(slow_idx), H_history(slow_idx), 15, 'b', 'filled', 'DisplayName', 'Slow Expert');
plot(time_steps, H_history, 'k:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
title('Water Height Using Hybrid Controller'); ylabel('Height (cm)'); grid on;
legend('Location', 'southeast'); ylim([0, 11]);

% --- SUBPLOT 2: APPLIED VOLTAGE ---
subplot(3,1,2); hold on;
% Faint connection line to show the continuous voltage path
plot(time_steps, u_history, 'w:', 'LineWidth', 0.5, 'HandleVisibility', 'off');

% Scatter dots colored by the active expert
scatter(time_steps(fast_idx), u_history(fast_idx), 10, 'r', 'filled');
scatter(time_steps(slow_idx), u_history(slow_idx), 10, 'b', 'filled');

yline(u_max, 'r:', 'Max Pump Limit (4.5V)', 'HandleVisibility', 'off');
title('Applied Hybrid Pump Voltage (Color-Coded)'); 
ylabel('Volts (V)'); 
grid on; 
ylim([-0.5, 5]);

% --- SUBPLOT 3: CONTROLLERS ---
subplot(3,1,3); hold on;
% Draw faint horizontal guide lines
yline(1, 'k:', 'HandleVisibility', 'off');
yline(2, 'k:', 'HandleVisibility', 'off');

% Plot discrete dots colored to match the experts
scatter(time_steps(fast_idx), active_controller(fast_idx), 10, 'r', 'filled');
scatter(time_steps(slow_idx), active_controller(slow_idx), 10, 'b', 'filled');

ylim([0.5, 2.5]); 
yticks([1, 2]); 
yticklabels({'1 (Fast Expert)', '2 (Slow Expert)'});
title('Active Controller Selection (Discrete Steps)'); 
xlabel('Time (s)'); 
ylabel('Expert ID'); 
grid on; 
set(gca, 'Color', 'w');

%% 6. Final Evaluation Metrics (The Hybrid Scorecard)
S_hybrid = stepinfo(H_history, time_steps, ref, 'SettlingTimeThreshold', 0.01);

fprintf('\n========================================\n');
fprintf('FINAL HYBRID SCORECARD\n');
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

%% ------------------------------------------------------------------------
%% HELPER FUNCTIONS
%% ------------------------------------------------------------------------

% 1. Standard Plant Dynamics
function dHdt = water_tank_dynamics(~, H, u, A, b, c)
    if H < 0, outflow = 0; else, outflow = (c / A) * sqrt(H); end
    inflow = (b / A) * u;
    dHdt = inflow - outflow;
end

% 2. The ode45 Predictive Safety Engine
function safe = check_fast_safety_ode(H_start, int_start, Kp, Ki, ref, u_max, u_min, A, b, c, T_horizon)
    % Set up the initial state for the future prediction [Height; Integrator]
    Y0 = [H_start; int_start];
    options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
    
    % Simulate the closed-loop physics 2.5 seconds into the future
    [~, Y_out] = ode45(@(t, Y) closed_loop_dynamics(t, Y, Kp, Ki, ref, u_max, u_min, A, b, c), [0, T_horizon], Y0, options);
    
    H_sim = Y_out(:, 1); % Extract just the height trajectory
    
    safe = true;
    if H_start <= ref
        % If climbing, check for violent overshoot
        if max(H_sim) > 10.005
            safe = false;
        end
    else
        % If recovering from above, check for violent undershoot (bounce)
        if min(H_sim) < 9.995
            safe = false;
        end
    end
end

% 3. Closed-Loop Continuous State-Space (Matches actual PI behavior)
function dYdt = closed_loop_dynamics(~, Y, Kp, Ki, ref, u_max, u_min, A, b, c)
    H = max(0, Y(1));
    I = Y(2);
    
    error = ref - H;
    u_test = (Kp * error) + (Ki * I);
    
    % Continuous Anti-Windup Logic
    if u_test <= u_max && u_test >= u_min
        dI = error;
    else
        dI = 0;
    end
    
    u_applied = max(u_min, min(u_max, u_test));
    
    outflow = (c / A) * sqrt(H);
    inflow = (b / A) * u_applied;
    dH = inflow - outflow;
    
    % Return the rate of change for both variables
    dYdt = [dH; dI];
end