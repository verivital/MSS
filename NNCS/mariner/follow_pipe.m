%% Generate a pipeline made of Np pipe segments for mariner to follow
addpath(genpath('C:\Users\manzand\Documents\MATLAB\MSS')); % add path to MATLAB
% Generate pipeline
pipe_gen; %could make this a function and have as inputs the number of segments
%pause;clc;
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
x(6) = deg2rad(angle_list(1));

% --- MAIN LOOP ---
n = round(t_f/h);               % number of samples
xout = zeros(n+1,10);  % memory allocation
ns = 1;        % number of pipes seen by ship
psi_ref = deg2rad(angle_list(1));              % desired heading angle

for i=1:n+1
    time = (i-1)*h;                   % simulation time in seconds
    r   = x(3);
    psi = x(6);
    

    % Where is the pipe? What is the pipe's angle?
    % p_list contains the position in x and y positions (end points of segments)
    % angle_list contains the orientation of these segments

    % Create perpendicular line to ship (defined by end points)
    [xs1,xs2,ys1,ys2] = perpendicular_to_ship(x(4),x(5),x(6)); 

    nointersection = false;
    while nointersection || (ns < length(angle_list))
        % intersection = function pending to write that returns true if lines intersect
        if intersecting_lines(xs1,xs2,ys1,ys2,p_list(ns,1),p_list(ns+1,1),p_list(ns,2),p_list(ns+1,2))
            psi_ref = angle_list(ns);
            nointersection = true;
            fprintf('Following pipe segment...  ns = %f, time = %f\n', ns, time);
            break;
        else
            fprintf('Ooooops, found no intersection  - ns = %f, time = %f\n', ns , time);
            fprintf('Ship location [%f,%f]\n\n', x(4),x(5));
            %Segment endpoints [x1=%f, x2=%f, y1=%f, y2=%f]\n',x(4),x(5),p_list(ns,1),p_list(ns+1,1),p_list(ns,2),p_list(ns+1,2));
            
            ns = ns + 1;
        end
        plot(x(4),x(5),'s'); %plot ship's current position
        pause;
    end

    % control system
    delta = -Kp*((psi-psi_ref)+Td*r);  % PD-controller

    % ship model
    [xdot,U] = mariner(x,delta);       % ship model, see .../gnc/VesselModels/

    % store data for presentation
    xout(i,:) = [time,x',U, psi_ref]; 

    % numerical integration
    x = euler2(xdot,x,h);             % Euler integration
end

hold on;
plot(xout(:,5),xout(:,6));
