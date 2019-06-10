%% Generate a pipeline made of Np pipe segments for mariner to follow
addpath(genpath('C:\Users\manzand\Documents\MATLAB\MSS')); % add path to MATLAB
% Generate pipeline
pipe_gen; %could make this a function and have as inputs the number of segments
% final simulation time (sec)
t_f = 600;
% sample time (sec)
h   = 0.1;  % UUV simulations work at 1 Hz

Kp = 1;      % controller P-gain
Td = 10;     % controller derivative time

% initial states:  x = [ u v r x y psi delta ]' 
x = zeros(7,1);
x(4) = x_init; % begin simulation with ship on top of pipeline (same initial position)
x(5) = y_init;

% --- MAIN LOOP ---
n = round(t_f/h);               % number of samples
xout = zeros(n+1,7+2+3);  % memory allocation
check_segment = 1;        % number of pipes seen by ship
psi_ref = 0;              % desired heading angle

for i=1:n+1
    time = (i-1)*h;                   % simulation time in seconds
    r   = x(3);
    psi = x(6);
    

    % Where is the pipe? What is the pipe's angle?
    % p_list contains the position in x and y positions (end points of segments)
    % angle_list contains the orientation of these segments

    % Create perpendicular line to ship
    perp_line = xxx; % some function I need to write. How should I represent this line, with end-points?

    nointersection = 1;
    while nointersection || (check_segment > length(angle_list))
        % intersection = function pending to write that returns true if lines intersect
        if intersection(perp_line,segment)  
            psi_ref = angle_list(check_segment);
            nointersection = 0;
        else
            check_segment = check_segment + 1;
        end
    end

    % control system
    delta = -Kp*((psi-psi_ref)+Td*r);  % PD-controller

    % ship model
    [xdot,U] = mariner(x,delta);       % ship model, see .../gnc/VesselModels/

    % store data for presentation
    xout(i,:) = [time,x',U]; 

    % numerical integration
    x = euler2(xdot,x,h);             % Euler integration
end


