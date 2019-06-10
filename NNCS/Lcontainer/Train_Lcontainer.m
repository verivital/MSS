%% Collect data MSS simulations
% Run Simulink model and collect data
% set the solver and simulation options 
clc;clear
%opt = simset('solver','ode4','FixedStep',0.02);

%% Generate training data
s = 120; % number of simulation I want to record
for i=1:s
    %rng(1); %set the seed for reproducible results
    psi_ref = 0.18*rand - 0.09; %different psi_ref for each simulation
    %Same initial conditions for every simulation
    % [T,X] = sim('Mountain_car',[0 15],opt); %simulate system and record data
    [T,X] = sim('Lcontainer_sim',[0 350]); %simulate system and record data
    % Create data object for each iteration
    d_plant = iddata(xdot,[x_in ui_in],0.1);
    d_nn = iddata(ui_out,[psi_ref_out psi_out r_out],0.1);
    d_tr = iddata(x_out,[xdot x_in h_t],0.1);
    % data = iddata(out1,in1,Ts);
    % Merge all simulations together
    if i>1
        data_plant = merge(data_plant,d_plant);
        data_nn = merge(data_nn,d_nn);
        data_tr = merge(data_tr,d_tr);
    else
        data_plant = merge(d_plant);
        data_nn = merge(d_nn);
        data_tr = merge(d_tr);        
    end
end

clearvars -except data_plant data_nn data_tr s;
%save('/home/musaup/Documents/MATLAB/diego/MSSdata/data_Lcontainer','data_plant', 'data_nn', 'data_tr'); %Lab GPU

%% Train Neural Network controller with only relus and linear function
% % Load data 
% d_nn = data_nn';
% out = d_nn.y;
% in = d_nn.u;
% for i=1:s
%     in{i} = in{i}';
%     out{i} = out{i}';
% end
% net_nn = network(1,3,[1;1;1],[1;0;0],[0 0 0;1 0 0;0 1 0],[0,0,1]); %4 inputs,2 layers, 1 output
% 
% % add the rest of structure
% net_nn.inputs{1}.size = 3; % size of inputs
% net_nn.layers{1}.size = 10; %size of layers
% net_nn.layers{2}.size = 10;
% net_nn.layers{3}.size = 1;
% net_nn.layers{1}.transferFcn = 'poslin'; %poslin = relu
% net_nn.layers{2}.transferFcn = 'poslin'; %poslin = relu
% net_nn.layers{3}.transferFcn = 'purelin'; % purelin = linear
% net_nn.initFcn = 'initlay';
% net_nn.trainFcn = 'trainbr'; %Bayesian regularization
% net_nn.layers{1}.initFcn = 'initnw';
% net_nn.layers{2}.initFcn = 'initnw';
% net_nn.layers{3}.initFcn = 'initnw';
% %net.inputWeights{1}.delays = 0:1;
% 
% %Store the output simulations
% % y1 = net_nn(in);
% net_nn = init(net_nn);
% net_nn = train(net_nn,in,out);
% % y2 = net_nn(in);
% % out = cell2mat(out);

%% Train Neural Network plant with only relus and linear function
% Load data 
d_p = data_plant';
out = d_p.y;
in = d_p.u;
for i=1:s
    in{i} = in{i}';
    out{i} = out{i}';
end
net_p = network(1,3,[1;1;1],[1;0;0],[0 0 0;1 0 0;0 1 0],[0,0,1]);

% add the rest of structure
net_p.inputs{1}.size = 10; % size of inputs
net_p.layers{1}.size = 50; %size of layers
net_p.layers{2}.size = 50;
net_p.layers{3}.size = 9; %& outputs
net_p.layers{1}.transferFcn = 'poslin'; %poslin = relu
net_p.layers{2}.transferFcn = 'poslin'; %poslin = relu
net_p.layers{3}.transferFcn = 'purelin'; % purelin = linear
net_p.initFcn = 'initlay';
net_p.trainFcn = 'trainlm'; %Bayesian regularization
net_p.trainParam.epochs = 10000;
net_p.trainParam.max_fail = 10;
net_p.trainParam.goal = 0.001;
net_p.layers{1}.initFcn = 'initnw';
net_p.layers{2}.initFcn = 'initnw';
net_p.layers{3}.initFcn = 'initnw';
%net.inputWeights{1}.delays = 0:1;

%Store the output simulations
% y1 = net_p(in);
net_p = init(net_p);
net_p = train(net_p,in,out,'useGPU','yes','showResources','yes'); %change to no when trining at lab
% y2 = net_p(in);
% out = cell2mat(out);

% %% Train Neural Network transform with only relus and linear function
% % Load data 
% d = data_tr';
% out = d.y;
% in = d.u;
% for i=1:s
%     in{i} = in{i}';
%     out{i} = out{i}';
% end
% net_tr = network(1,3,[1;1;1],[1;0;0],[0 0 0;1 0 0;0 1 0],[0,0,1]);
% 
% % add the rest of structure
% net_tr.inputs{1}.size = 19; % 15 inputs
% net_tr.layers{1}.size = 10; %size of layers
% net_tr.layers{2}.size = 10;
% net_tr.layers{3}.size = 9; %& outputs
% net_tr.layers{1}.transferFcn = 'poslin'; %poslin = relu
% net_tr.layers{2}.transferFcn = 'poslin'; %poslin = relu
% net_tr.layers{3}.transferFcn = 'purelin'; % purelin = linear
% net_tr.initFcn = 'initlay';
% net_tr.trainFcn = 'trainbr'; %Bayesian regularization
% net_tr.layers{1}.initFcn = 'initnw';
% net_tr.layers{2}.initFcn = 'initnw';
% net_tr.layers{3}.initFcn = 'initnw';
% %net.inputWeights{1}.delays = 0:1;
% 
% %Store the output simulations
% % y1 = net_p(in);
% net_tr = init(net_tr);
% net_tr = train(net_tr,in,out);
% % y2 = net_p(in);
% % out = cell2mat(out);
% 
% %% Save the net data in a dictionary
% % mxv = [-0.4 0.07];
% % mnv = [-0.6 -0.07];
% % 
% % W = {{net.IW{1} net.LW{2} net.LW{6}}};
% % w = W{1};
% % b = {{net.b{1} net.b{2} net.b{3}}};
% % ni = size(w{1},2);
% % no = net.output.size;
% % nl = length(net.layers);
% % nn = prod(size(w{1}))+prod(size(w{2}))+prod(size(w{3}));
% % ls = [net.layers{1}.size,net.layers{2}.size,net.layers{3}.size];
% 
% % Define a dictionary with net's info
% % nnetwork = struct('number_of_outputs',no,'number_of_inputs',ni,...
% %     'number_of_layers',nl,'number_of_weights',nn,'W',W,...
% %     'b',b,'layer_size',ls,'act_functions',{{'relu','relu','linear'}},...
% %     'number_of_neurons',sum(ls),'max',mxv,'min',mnv);
% % save the net as Tran's tool
% % save('MountainCar_ReluController','nnetwork');