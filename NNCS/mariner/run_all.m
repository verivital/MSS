%% Run all mpossible mariner simulations
% For now, it runs all simulations (simple and sonar) with the PID controller and the
% neural network controller

% The neural network controller to be used is controller_mariner_3in.mat

% Simulation 1 - Simple with PID controller
sim1 = follow_pipeline_simple(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6]);

% Simulation 2 - Simulated sonar with PID controller
sim2 = follow_pipeline_sonar(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6]);

% Simulation 3 - Simple with NN controller
sim3 = follow_pipeline_simple_NN(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6]);

% Simulation 4 - Simulated sonar with NN controller
sim4 = follow_pipeline_sonar_NN(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6]);