% title: WaterTank_imitation_pb
% trains & falsify the NN

classdef WaterTank_imitation_pb < imitation_pb
    methods
        function pb = WaterTank_imitation_pb
            % Creates class instance with name
            pb = pb@imitation_pb('WaterTank_hybrid');
            
            %% State space definition (What the NN sees)
            pb.states.H.nominal = 0.0;
            pb.states.integral_sum.nominal = 0.0;
            
            % Initial ranges (Falsifier search space)
            pb.states.H.range_init = [0.0, 0.5];
            pb.states.integral_sum.range_init = [0.0, 0.0];
            
            % Input ranges (What the NN outputs)
            pb.controls.u_applied.range = [0.0; 4.5];
            
            % Grid resolution - Hardcoded to prevent Duplicate Gatekeeper rejection!
            pb.states.H.grid_res = 0.005;
            pb.states.integral_sum.grid_res = 0.1;
            
            %% Control
            pb.mpc_controlfn = @(x, options) expert_watertank_control(x, options);
                                    
            %% Simulation
            Ts = 0.1;
            Tf = 80;
            time = 0:Ts:Tf;
            pb.time = time;
            
            % Physics wrapper
            pb.sim_fn = @(Sys, time, p, controlfn)(sim_breach_watertank(Sys, time, p, controlfn));
            pb.plot_sim = @(t, X, ax) plot_sim_watertank(t, X, ax);
            
            %% Requirements
            % Overshoot <= 10.2cm globally. Steady state between 9.9 and 10.1cm from t=70s to 80s
            pb.phi = BreachRequirement('alw (H <= 10.2) and alw_[70, 80] (H >= 9.9 and H <= 10.1)');
        end
        
        function options = setup_training_options(this, Bvalid)
            if nargin==1
                % validation baseline data is created
                Bvalid = this.get_mpc_traces('num_corners',0,'num_quasi_random',10,'seed', 2000);
            end
            [in_valid, out_valid] = this.prepare_training_data(Bvalid);
            valid_cell_array = {in_valid,out_valid};
            options = trainingOptions('adam', ...
                'Verbose', false, ...
                'Plots','training-progress',... 
                'Shuffle', 'every-epoch', ...
                'MiniBatchSize', 64, ...
                'ValidationData', valid_cell_array, ...
                'InitialLearnRate', 1e-3, ...
                'ExecutionEnvironment', 'cpu', ...
                'GradientThreshold', 10, ...
                'MaxEpochs', 50 ...
                );
        end
        
        function layers = setup_layers(this)
            numObservations = numel(fieldnames(this.states));
            numActions = numel(fieldnames(this.controls));
            hiddenLayerSize = 64;
            umax = 4.5; 
            layers = [
                featureInputLayer(numObservations,'Normalization','none','Name','observation')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc1')
                reluLayer('Name','relu1')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc2')
                reluLayer('Name','relu2')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc3')
                reluLayer('Name','relu3')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc4')
                reluLayer('Name','relu4')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc5')
                reluLayer('Name','relu5')
                fullyConnectedLayer(hiddenLayerSize,'Name','fc6')
                reluLayer('Name','relu6')
                fullyConnectedLayer(numActions,'Name','fcLast')
                % NN squashes outputs strictly between 0.0 and 1.0
                sigmoidLayer('Name','sigmoidLast')
                scalingLayer('Name','ActorScaling','Scale',umax)
                regressionLayer('Name','routput')];
        end
        
        function str = get_result_file_name(~, ~)
            fileName = ['Res_WaterTank_' datestr(now, 'dd-mmm-yyyy-HH_MM_SS')];
            str = fullfile('watertank_NN', fileName);
        end
    end
end