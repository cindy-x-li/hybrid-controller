% title: sim_breach_watertank.m
% Breach needs 4 outputs: time array, state trajectory matrix, parameter
% array of the starting conditions, and simulation status
function [time_out, X_traj, p, status] = sim_breach_watertank(Sys, time_span, p, controlfn)
    A = 100; b = 6.17; c = 4.39;
    H_ref = 10.0;
    time_step = 0.1;
    
    % --- GLOBAL VARIABLE RESET ---
    global prev_ctrl;
    prev_ctrl = 1; % Clean slate for every new simulation trace!
    
    % Extract Initial States from the Falsifier
    H_curr = p(Sys.DimX + 1);
    integral_sum = p(Sys.DimX + 2);
    
    X_traj = zeros(Sys.DimX, length(time_span));
    
    for k = 1:length(time_span)
        
        % BOTH Expert and NN now take the exact same 2-element input!
        x_in = [H_curr; integral_sum];
        
        try
            % ATTEMPT 1: The Expert Controller
            out = double(controlfn(x_in, []));
            u_applied = out(1);
            active_ctrl = double(out(2) >= 0.5);
            
            % Update the global variable for the next time step
            prev_ctrl = active_ctrl; 
        catch
            % ATTEMPT 2: The Neural Network
            out = double(controlfn(x_in, []));
            u_applied = out(1);
            
            % The NN doesn't output an active_ctrl.
            % The global prev_ctrl stays locked, ensuring a stable cap.
            active_ctrl = prev_ctrl; 
        end
        
        % --- TRACK DATA FOR BREACH ---
        % (Assuming your Sys.DimX is 3 or 4: [H, integral, u_applied, cost])
        % Notice prev_ctrl is totally hidden from Breach!
        tracked_data = [H_curr; integral_sum; u_applied; 0];
        X_traj(:, k) = tracked_data(1:Sys.DimX); 
        
        if k == length(time_span)
            break;
        end
        
        % --- PHYSICS UPDATE ---
        [~, H_out] = ode45(@(t, H) ((b/A)*u_applied) - ((c/A)*sqrt(abs(H))), [0, time_step], H_curr);
        H_curr = max(0, H_out(end));
        
        % --- DYNAMIC INTEGRAL CAPPING ---
        if active_ctrl >= 0.5
            Ki_current = 12.0; % Fast Mode
        else
            Ki_current = 11.0; % Slow Mode
        end
        
        u_ss = (c / b) * sqrt(H_ref);
        I_ideal = u_ss / Ki_current;
        I_max = I_ideal * 1.5;
        I_min = -I_max;
        
        error = H_ref - H_curr;
        integral_sum = integral_sum + (error * time_step);
        integral_sum = max(I_min, min(I_max, integral_sum)); 
    end
    
    time_out = time_span;
    status = 0; 
end