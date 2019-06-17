function [xout] = follow_pipeline_sonar(L,n_P,vehicle,n_s,n_x,n_y,n_psi,t_f,h,out_mat)
%%FOLLOW_PIPELINE_SONAR
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
% One example on how to run this code may be
% [xout] = follow_pipeline_sonar(160.93,25,@Mariner,7,4,5,6,10000,0.1,[3 6]);

% Generate pipeline
L_min = 10*L; 
L_max = L_min + 1500; %(m)
[p_list,angle_list,x_init,y_init] = gen_pipe(L_min,L_max,n_P);
hold on; % to plot vehicle trajectory on same graph

%controller parameters (may need to change this in future for NN)
Kp = 1;      % controller P-gain
Td = 10;     % controller derivative time

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

for i=1:n+1
    time = (i-1)*h;                   % simulation time in seconds
%     r   = x(3);
%     psi = x(6);

    % Where is the pipe? What is the pipe's angle?
    % p_list contains the position in x and y positions (end points of segments)
    % angle_list contains the orientation of these segments

    % Create perpendicular line to ship (defined by end points)
    [xs1,xs2,ys1,ys2] = perpendicular_to_ship(x(n_x),x(n_y),x(n_psi));
    %plot([xs1,xs2],[ys1,ys2],'-');

    nointersection = false;
    while nointersection || (ns < length(angle_list))
        % intersection = function pending to write that returns true if lines intersect
        if intersecting_lines(L,xs1,xs2,ys1,ys2,p_list(ns,1),p_list(ns+1,1),p_list(ns,2),p_list(ns+1,2))
            psi_ref = deg2rad(angle_list(ns));
            nointersection = true;
            fprintf('Following pipe segment...  ns = %f, time = %f\n', ns, time);
            break;
        else
            fprintf('Ooooops, found no intersection  - ns = %f, time = %f\n', ns , time);
            fprintf('Ship location (x,y,angle) [%f,%f,%f]\n\n', x(4),x(5),x(6));
            %Segment endpoints [x1=%f, x2=%f, y1=%f, y2=%f]\n',x(4),x(5),p_list(ns,1),p_list(ns+1,1),p_list(ns,2),p_list(ns+1,2));
            ns = ns + 1;
            pause;
            break;
        end
        %plot(x(4),x(5),'s'); %plot ship's current position
        %pause;
    end

    % output mat
    for j=1:length(out_mat)
       out(j) = out_mat(j); 
    end
    
    % control system
    delta = -Kp*((x(out(2))-psi_ref)+Td*x(out(1)));  % PD-controller

    % ship model
    xdot = vehicle(x,delta);       % ship model, see .../gnc/VesselModels/

    % store data for presentation
    xout(i,:) = [time,x', psi_ref]; 

    % numerical integration
    x = euler2(xdot,x,h);             % Euler integration
    if mod(time,50) == 0
        %plot(x(4),x(5),'s'); % ship location
        fprintf('Segment #%d currently following at time %f with ref angle %f\n',round(ns),time, rad2deg(psi_ref))
        %pause;
    end
%     pause;
    if ns == 25
        break;
    end
end

end

