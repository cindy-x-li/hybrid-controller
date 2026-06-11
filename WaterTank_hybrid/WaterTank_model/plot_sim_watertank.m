% required by the imitation_pb.m

function plot_sim_watertank(B0)
    figure('Name', 'Water Tank Imitation Traces', 'Position', [100, 100, 800, 600], 'Color', 'w');

    % --- SETUP SUBPLOT 1 (Height) ---
    subplot(2,1,1); hold on; grid on;
    yline(10.0, 'k-', 'Target (10.0 cm)', 'LineWidth', 1.5);
    yline(10.2, 'r:', 'Max Overshoot (10.2 cm)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    yline(10.1, 'g--', 'Upper Bound (10.1 cm)', 'LineWidth', 1.0);
    yline(9.9, 'g--', 'Lower Bound (9.9 cm)', 'LineWidth', 1.0);
    ylabel('Height (cm)'); title('Water Level (10 Random Traces)');
    ylim([0, 12]);

    % --- SETUP SUBPLOT 2 (Voltage) ---
    subplot(2,1,2); hold on; grid on;
    yline(4.5, 'r:', 'Max Voltage (4.5V)', 'LabelHorizontalAlignment', 'left');
    yline(0.0, 'r:', 'Min Voltage (0V)', 'LabelHorizontalAlignment', 'left');
    ylabel('Pump Volts (V)'); xlabel('Time (s)'); title('Pump Voltage');
    ylim([-0.5, 5]);

    % --- PLOT ALL TRACES ---
    % Loop through the Teacher's Logbook inside B0 and plot each trace
    for i = 1:length(B0.P.traj)
        t = B0.P.traj{i}.time;
        H = B0.P.traj{i}.X(1,:);       % Row 1 is Water Height
        u = B0.P.traj{i}.X(4,:);       % Row 4 is Applied Voltage
        
        subplot(2,1,1);
        plot(t, H, 'b-', 'LineWidth', 1.0);
        
        subplot(2,1,2);
        plot(t, u, '-', 'Color', [0.8500 0.3250 0.0980 0.5], 'LineWidth', 1.0);
    end
end