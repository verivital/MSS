%% Run all mpossible mariner simulations
% For now, it runs all simulations (simple and sonar) with the PID controller and the
% neural network controller. All simulations generate different pipelines

% The neural network controller to be used is controller_mariner_3in.mat

% Simulation 1 - Simple with PID controller
fi = figure;
'Simulation 1'
sim1 = follow_pipeline_simple(1.98,25,@mariner_uuv,7,4,5,6,10000,0.1,[3 6]);
'Press any key for moving into next simulation'
title('Simulation 1 - Waypoints and ground truth (PID)');
xlabel('x positon (m)');
ylabel('y position (m)');
saveas(fi,'figures/sim1','png');
pause;
clc;

% Simulation 2 - Simulated sonar with PID controller
fi = figure;
'Simulation 2'
sim2 = follow_pipeline_sonar(1.98,25,@mariner_uuv,7,4,5,6,10000,0.1,[3 6]);
'Press any key for moving into next simulation'
title('Simulation 2 - Simulated sonar and ground truth (PID)');
xlabel('x positon (m)');
ylabel('y position (m)');
saveas(fi,'figures/sim2','png');
pause;
clc;

% Simulation 3 - Simple with NN controller
fi = figure;
'Simulation 3'
sim3 = follow_pipeline_simple_NN(1.98,25,@mariner_uuv,7,4,5,6,10000,0.1,[3 6],'controller_mariner_3in_net.mat');
'Press any key for moving into next simulation'
title('Simulation 3 - Waypoints and ground truth (NN)');
xlabel('x positon (m)');
ylabel('y position (m)');
saveas(fi,'figures/sim3','png');
pause;
clc;

% Simulation 4 - Simulated sonar with NN controller
fi = figure;
'Simulation 4'
sim4 = follow_pipeline_sonar_NN(1.98,25,@mariner_uuv,7,4,5,6,10000,0.1,[3 6],'controller_mariner_3in_net.mat');
title('Simulation 4 - Simulated sonar and ground truth (NN)');
xlabel('x positon (m)');
ylabel('y position (m)');
saveas(fi,'figures/sim4','png');
'End of simulations'

