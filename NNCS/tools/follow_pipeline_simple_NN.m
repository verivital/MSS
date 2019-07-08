function [xout] = follow_pipeline_simple_NN(L,n_P,vehicle,n_s,n_x,n_y,n_psi,t_f,h,out_mat,NN)
%%FOLLOW_PIPELINE_GENERAL 
% Given a determined vehicle dynamics, it will follow a pipeline (created by pipe_gen) either via waypoints or sonar simulation
% [xout] = follow_pipeline_general(L,vehicle,t_f,h,mode)
%   L = length of vehicle (m)
%   p_N = number of pipe segments to generate
%   vehicle = dynamics of vehicle (specified as vehicle.m)
%   n_s = number of states in dynamics
%   n_x = state # corresponding to x position
%   n_y = state # corresponding to y position
%   n_psi = state # corresponding to ship orientation (psi)
%   t_f = final time step for simulation
%   h = time step (s)
%   out_mat = matrix with the number and location of plant outputs into controller
%   
%
% One example on how to run this code may be
% [xout] = follow_pipeline_simple_NN(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6],'controller_mariner_3in_net.mat');

% Generate pipeline
L_min = 10*L; 
L_max = L_min + 1500; %(m)
[p_list,angle_list,x_init,y_init] = gen_pipe(L_min,L_max,n_P);
hold on; % to plot vehicle trajectory on same graph

%controller parameters (may need to change this in future for NN)
load(NN);

% initial states:  x = [ u v r x y psi delta ]' (for mariner)
x = zeros(n_s,1);
x(n_x) = x_init; % begin simulation with ship on top of pipeline (same initial position)
x(n_y) = y_init;
x(n_psi) = deg2rad(angle_list(1));

% --- MAIN LOOP ---
n = round(t_f/h);               % number of samples
xout = zeros(n+1,n_s+2);  % memory allocation
ns = 1;        % number of pipes seen by ship
psi_ref = deg2rad(angle_list(1));              % desired heading angle
i = 1;
time = 0;
xy_norm = zeros(n,2);  %memory allocation for distance between ship and objective

while ns < length(p_list)+1 - 1 && i < n - 1
    xy_norm(i,:) = [pdist([x(n_x),x(n_y);p_list(ns+1,1), p_list(ns+1,2)]),ns];
    if pdist([x(n_x),x(n_y);p_list(ns+1,1), p_list(ns+1,2)]) < 100
        ns = ns+1;
        fprintf('Segment #%d currently following at time %f\n',int64(ns),time)
    elseif xy_norm(i,1) > xy_norm(max(1,i-1),1) && xy_norm(i,2) == xy_norm(max(1,i-10),2)
        ns = ns+1;
        fprintf('Getting too far from objective\n');
        fprintf('Changed to follow segment #%d at time %f\n',int64(ns),time)
    end
    % Calculate reference psi
    psi_ref = deg2rad(angle_list(ns));

    time = (i-1)*h;                   % simulation time in seconds
    
    % output from plant to controller (output_mat)
    % Keep this for now, but need to generalize so that user can specify
    % which states to feed into the controller

    % output mat
    for j=1:length(out_mat)
       out(j) = out_mat(j); 
    end
%     r   = x(out(1));
%     psi = x(out(2));
    
    % control system
    % delta = -Kp*((x(out(2))-psi_ref)+Td*x(out(1)));  % PD-controller
    delta = net([out(1);out(2);-psi_ref]);

    % ship model
    [xdot] = vehicle(x,delta);       % ship model, see .../gnc/VesselModels/

    % store data for presentation
    xout(i,:) = [time,x',psi_ref]; 

    % numerical integration
    x = euler2(xdot,x,h);             % Euler integration
    %pause;
    i = i+1;
    if mod(time,10) == 0
        plot(x(n_x),x(n_y),'s'); % ship location
        fprintf('Segment #%d currently following at time %f with ref angle %f\n',round(ns),time, rad2deg(psi_ref))
        %pause;
    end
end

end


