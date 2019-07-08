%% Run all mpossible mariner simulations
% For now, it runs all simulations (simple and sonar) with the PID controller and the
% neural network controller. All simulations generate different pipelines

% The neural network controller to be used is controller_mariner_3in.mat

% Simulation 1 - Simple with PID controller
'Simulation 1';
sim1 = follow_pipeline_simple(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6]);
'Press any key for moving into next simulation';
pause;
clc;

% Simulation 2 - Simulated sonar with PID controller
figure;
'Simulation 2';
sim2 = follow_pipeline_sonar(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6]);
'Press any key for moving into next simulation';
pause;
clc;

% Simulation 3 - Simple with NN controller
figure;
'Simulation 3';
sim3 = follow_pipeline_simple_NN(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6],'controller_mariner_3in_net.mat');
'Press any key for moving into next simulation';
pause;
clc;

% Simulation 4 - Simulated sonar with NN controller
figure;
'Simulation 4';
sim4 = follow_pipeline_sonar_NN(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6],'controller_mariner_3in_net.mat');
'End of simulations';