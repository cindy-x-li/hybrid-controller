% returns voltage & controller choice
function out = expert_watertank_control(x, ~)
    % Declare the global variable
    global prev_ctrl;
    
    % Failsafe: If it hasn't been initialized by the simulation yet, default to Fast
    if isempty(prev_ctrl)
        prev_ctrl = 1;
    end

    % x is now strictly the 2-element state vector exactly as the NN sees it!
    H_current = x(1);
    shared_integral = x(2);
    
    % Plant & Controller Parameters
    A = 100; b = 6.17; c = 4.39;
    u_max = 4.5; u_min = 0.0; ref = 10.0;
    Kpf = 7; Kif = 12;
    Kps = 25; Kis = 11;
    Ts = 0.1;
    epsilon = 0.2; 
    
    error = ref - H_current;
    
    % --- 1-STEP PREDICTION ---
    U_f_raw = (Kpf * error) + (Kif * shared_integral);
    U_s_raw = (Kps * error) + (Kis * shared_integral);
    
    U_f_test = max(u_min, min(u_max, U_f_raw));
    U_s_test = max(u_min, min(u_max, U_s_raw));
    
    outflow = (c / A) * sqrt(max(0, H_current));
    H_pred_f = H_current + (((b / A) * U_f_test - outflow) * Ts);
    H_pred_s = H_current + (((b / A) * U_s_test - outflow) * Ts);
    
    Error_f = abs(ref - H_pred_f);
    Error_s = abs(ref - H_pred_s);
    
    % --- SUPERVISORY LOGIC ---
    if Error_f <= epsilon && Error_s <= epsilon
        active_ctrl = prev_ctrl; % Rule 1: Stickiness
    else
        if Error_f <= Error_s 
            active_ctrl = 1; % Rule 2: Greedy Fast
        else
            active_ctrl = 0; % Rule 2: Greedy Slow
        end
    end
    
    % --- CALCULATION ---
    if active_ctrl == 1
        u_raw = (Kpf * error) + (Kif * shared_integral);
    else
        u_raw = (Kps * error) + (Kis * shared_integral);
    end
    
    u_applied = max(u_min, min(u_max, u_raw));
    
    % Output both the voltage and the choice
    out = [u_applied; active_ctrl];
end