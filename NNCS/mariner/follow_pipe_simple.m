%% Generate a pipeline made of Np pipe segments for mariner to follow
% It follows the angle of a pipe until the ship s close enough to the
% begining of the next pipe segment, and updates its desired heading angle
% to that of the next segment
addpath(genpath('C:\Users\manzand\Documents\MATLAB\MSS')); % add path to MATLAB
% Generate pipeline
pipe_gen; %could make this a function and have as inputs the number of segments
hold on;
%pause;clc;
% final simulation time (sec)
t_f = 100;
% sample time (sec)
h   = 0.1;  % UUV simulations work at 1 Hz

Kp = 1;      % controller P-gain
Td = 10;     % controller derivative time

% initial states:  x = [ u v r x y psi delta ]' 
x = zeros(7,1);
x(4) = x_init; % begin simulation with ship on top of pipeline (same initial position)
x(5) = y_init;
x(6) = deg2rad(angle_list(1));

% --- MAIN LOOP ---
n = round(t_f/h);               % number of samples
xout = zeros(n+1,10);  % memory allocation
ns = 1;        % number of pipes seen by ship
psi_ref = deg2rad(angle_list(1));              % desired heading angle
i = 1;
time = 0;
%xy_norm = zeros(6001,1);

while ns < length(p_list) && time < n - 1
    %psi_ref = deg2rad(angle_list(ns));
    if pdist([x(4),x(5);p_list(ns+1,1), p_list(ns+1,2)]) < 100
        ns = ns+1;
        fprintf('Segment number currently following... %f at time %f\n',round(ns),time)
    end
    % Calculate reference psi
    psi_ref = deg2rad(angle_list(ns));
    xy_norm(i) = pdist([x(4),x(5);p_list(ns+1,1), p_list(ns+1,2)]);

    time = (i-1)*h;                   % simulation time in seconds
    r   = x(3);
    psi = x(6);

    % control system
    delta = -Kp*((psi-psi_ref)+Td*r);  % PD-controller

    % ship model
    [xdot,U] = mariner(x,delta);       % ship model, see .../gnc/VesselModels/

    % store data for presentation
    xout(i,:) = [time,x',U, psi_ref]; 

    % numerical integration
    x = euler2(xdot,x,h);             % Euler integration
    %pause;
    i = i+1;
end

hold on;
plot(xout(:,5),xout(:,6));