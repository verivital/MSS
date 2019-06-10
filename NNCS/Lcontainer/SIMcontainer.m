clc;clear
% SIMcontainer User editable script for simulation of the container ship under feedback control
%              Both the linear model Lcontainer.m and nonlinear model container.m are simulated.
%
% Calls:       Lcontainer.m and euler2.m
%
% Author:      Thor I. Fossen
% Date:        2018-07-21
% Revisions: 

disp('Simulating container.m and Lcontainer.m under PD-control with psi_ref=[-15 to 15] (deg) ...')

t_f = 400;   % final simulation time (sec)
h   = 0.1;   % sample time (sec)

Kp = 1;      % controller P-gain
Td = 10;     % controller derivative time

% initial states:
x  = [7 0 0 0 0 0 0 0 0]';   % x = [u v r x y psi p phi delta]'    

% --- MAIN LOOP ---
N = round(t_f/h);               % number of samples
xout = zeros(N+1,length(x)+1);  % memory allocation
xin = zeros(N+1,length(x));   % memory allocation
xderi = zeros(N+1,length(x));   % memory allocation
sims = 150;                     % number of simulations
p_ref = zeros(N+1,1);
delta_cont = zeros(N+1,1);


for j = 1:sims
    % initial states:
    x  = [7 0 0 0 0 0 0 0 0]';   % x = [u v r x y psi p phi delta ]'    
    psi_ref = 0.18*rand - 0.09;          % desired heading
    
    for i=1:N+1
        time = (i-1)*h;                   % simulation time in seconds
        xin(i,:) = x';
        r   = x(3);
        psi = x(6);

        % control system
        p_ref(i,:) = psi_ref;
        delta_c = -Kp*((psi-psi_ref)+Td*r);  % PD-controller
        delta_cont(i,:) = delta_c;
        % n_c = 70;                          % Only for nonlinear container

        % ship model
        %xdot_nn = net([xin delta_c]);
        [xdot,U] = Lcontainer(x,delta_c);  % ship model, see .../gnc/VesselModels/
        xderi(i,:) = xdot; 

        % numerical integration
        x  = euler2(xdot,x,h);             % Euler integration   
        
        % store data for presentation
        xout(i,:) = [x',U]; 
        
    end
    % time-series
    uo(:,:,j)     = xout(:,1);
    vo(:,:,j)     = xout(:,2); 
    ro(:,:,j)     = xout(:,3); %- min(xout(:,3)))/(max(xout(:,3))-min(xout(:,3)));          
    xo(:,:,j)     = xout(:,4);   
    yo(:,:,j)     = xout(:,5);
    psio(:,:,j)   = xout(:,6);% - min(xout(:,6)))/(max(xout(:,6))-min(xout(:,6)));
    po(:,:,j)     = xout(:,7);
    phio(:,:,j)   = xout(:,8);
    deltao(:,:,j) = xout(:,9);
    Uo(:,:,j)     = xout(:,10);
    prefo(:,:,j)  = p_ref; %-min(p_ref))/(max(p_ref)-min(p_ref));
    conto(:,:,j)  = delta_cont;
    xderio(:,j,:) = xderi';
    xino(:,j,:)   = xin'; %-min(xin)')./(max(xin)'-min(xin)');

end

%Reshape data
u     = reshape(uo,1,(N+1)*sims); % pertubed surge velocity about Uo (m/s)
v     = reshape(vo,1,(N+1)*sims); % pertubed sway velocity about zero (m/s)         
r     = reshape(ro,1,(N+1)*sims); % pertubed yaw velocity about zero (rad/s)   
x     = reshape(xo,1,(N+1)*sims); % position in x-direction (m)
y     = reshape(yo,1,(N+1)*sims); % position in y-direction (m)
psi   = reshape(psio,1,(N+1)*sims); % pertubed yaw angle about zero (rad)
p     = reshape(po,1,(N+1)*sims);
phi   = reshape(phio,1,(N+1)*sims);
delta = reshape(deltao,1,(N+1)*sims); % actual rudder angle (rad)
U     = reshape(Uo,1,(N+1)*sims); % speed transform
xin   = reshape(xino,9,(N+1)*sims); % states inputs to calculate speed and forces
psi_ref = reshape(prefo,1,(N+1)*sims); % Reference psi
%error = reshape(erro,1,(N+1)*sims);
cont_out = reshape(conto,1,(N+1)*sims);
xdot  = reshape(xderio,9,(N+1)*sims);

% Normalize data
datatr = [xin; u; v; r; x; y; psi; p; phi; delta; U; psi_ref; cont_out; xdot];
datatr = (datatr' - min(datatr'))./(max(datatr')-min(datatr'));
datatr = datatr';
% Substitute NaN by 0 and 1, depending on original data
datatr(1,:) = 1; %Original constant value = 7
datatr(10,:) = 1; %Original constant value = 7
datatr(22,:) = 0; %Original constant value = 0

% Shuffle the data before training any NN
[m,n] = size(datatr);
idx = randperm(n);
data = datatr;
for i = 1:(size(datatr,1))
    data(i,idx) = data(i,:);
end

%% Train a NN to substitute the explicit plant (Compute xdot from previous state + input)
layers = [100 100 100 100];
net = feedforwardnet(layers,'trainlm');
net.inputs{1}.processFcns = {};
net.outputs{length(layers)+1}.processFcns = {};
net.inputs{1}.size = 10;
net.layers{length(layers)+1}.size = 9;
net.layers{1}.transferFcn = 'poslin'; %poslin = relu
net.layers{2}.transferFcn = 'poslin'; %poslin = relu
net.layers{3}.transferFcn = 'poslin'; %poslin = relu
net.layers{4}.transferFcn = 'poslin'; %poslin = relu
net.layers{5}.transferFcn = 'purelin'; % purelin = linear
net.trainParam.epochs = 10000;
net.trainParam.max_fail = 10;
net.trainParam.mu_max = 10e20;
net.trainParam.goal = 0.0000000001;
net.performFcn = 'mse';%help nnperformance to see list of options
net.trainParam.min_grad = 1e-11;

% datatr = [xin(9-dim); u, v, r, x, y, psi, p; phi; delta, U, psi_ref; cont_out; xdot(9-dim)];
in = {[data(1:9,1:250000); data(21,1:250000)]};
out = {data(22:end,1:250000)};
in_test = {[data(1:9,end-10000:end); data(21,end-10000:end)]};
out_test = {data(22:end,end-10000:end)};

net = train(net,in,out,'useGPU','yes','useParallel','no','showResources','yes');
outnet = net(in_test);
perf = perform(net,out_test,outnet)

