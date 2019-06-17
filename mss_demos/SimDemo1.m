echo on
% SIMDEMO1    User editable script for simulation of the 
%              mariner class vessel under feedback control
%
% Calls:      mariner.m and euler2.m
%
% Author:     Thor I. Fossen
% Date:       19th July 2001
% Revisions: 

echo off 
disp('Simulating mariner.m under PD-control with psi_ref=5 (deg) ...')

t_f = 45000;   % final simulation time (sec)
h   = 0.1;   % sample time (sec)

Kp = 1;      % controller P-gain
Td = 10;     % controller derivative time

% initial states:  x = [ u v r x y psi delta ]' 
x = zeros(7,1);   

% --- MAIN LOOP ---
N = round(t_f/h);               % number of samples
xout = zeros(N+1,length(x)+2);  % memory allocation
%tvec = [0 : h : t_f];

v_psi_ref = zeros(N+1,2);

psi_ref = 180 * (pi/180);

for i=1:N+1,
    time = (i-1)*h;                   % simulation time in seconds

    r   = x(3);
    psi = x(6);
    
    % control system
    if time < 18000
        if mod(time,300) == 0
            psi_ref = -psi_ref;              % desired heading
        end
    else
        if time == 18000
            psi_ref = 270 * (pi/180);
        end
        if time == 21000
            psi_ref = 360 * (pi/180);
        end
        if mod(time,300) == 0
            psi_ref = -psi_ref;              % desired heading
        end
        %psi_ref = 45*(pi/180);
    end
    delta = -Kp*((psi-psi_ref)+Td*r);  % PD-controller

    % ship model
    [xdot,U] = mariner(x,delta);       % ship model, see .../gnc/VesselModels/
    
    % store data for presentation
    xout(i,:) = [time,x',U]; 
    v_psi_ref(i,:) = [time,psi_ref]; 
    
    % numerical integration
    x = euler2(xdot,x,h);             % Euler integration
end

% time-series
t     = xout(:,1);
u     = xout(:,2); 
v     = xout(:,3);          
r     = xout(:,4)*180/pi;   
x     = xout(:,5);
y     = xout(:,6);
psi   = xout(:,7)*180/pi;
delta = xout(:,8)*180/pi;
U     = xout(:,9);

% plots
figure(1)
plot(y,x),grid,axis('equal'),xlabel('East'),ylabel('North'),title('Ship position')

figure(3);
plot3(y,x,t);

figure(2)
subplot(221),plot(t,r),xlabel('time (s)'),title('yaw rate r (deg/s)'),grid
subplot(222),plot(t,U),xlabel('time (s)'),title('speed U (m/s)'),grid
subplot(223),plot(t,psi),xlabel('time (s)'),title('yaw angle \psi (deg)'),grid
subplot(224),plot(t,delta),xlabel('time (s)'),title('rudder angle \delta (deg)'),grid
