% test_watertank.m

% Initialize the class
init_watertank;

% Create the nominal Breach system
B0 = pb_watertank.create_nominal();

% Sample the domain and simulate
B0.SampleDomain(10);
B0.Sim();

% Plot the traces to verify the Expert controller works inside the framework
plot_sim_watertank(B0);