function [p_list,angle_list,x_init,y_init] = gen_pipe(L_min,L_max,p_N)
%GEN_PIPE Generates a pipeline and returns the intersection points of pipes
%(p_list) and the angles that the pipes makes wrt x-axis (angle_list)
%   [p_list,oangle_list] = gen_pipe(L_min,L_max,n)
%   L_min = minimum length of pipe segments (m)
%   L_max = maximum length of pipe segments (m)
%   p_N = number of pipe segments to generate

% Range where pipeline begins
x_range = [-100,100];
y_range = [-25,100];

% Initial point of pipeline
x_init = x_range(1) + (x_range(2) - x_range(1)).*rand(1,1);
y_init = y_range(1) + (y_range(2) - y_range(1)).*rand(1,1);
p_init = [x_init,y_init];

% List of begin-end points for pipes
p_list = zeros(p_N,2);
p_list(1,:) = p_init;

% List of angles
angle_list = zeros(p_N-1,1);
%angle_list(1,:) = 0; % degrees

% Angle limits 
theta_min = deg2rad(-180);
theta_max = deg2rad(180);

for i_p = 2 : p_N
    %clc;
    L_x = L_min + (L_max - L_min) * rand(1,1);
    L_y = L_min + (L_max - L_min) * rand(1,1);
    
    bad_angle = 1;
    while bad_angle
        %'trying new points'
        theta = theta_min + (theta_max - theta_min) * rand(1,1);
        rot_theta = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        
        p_next = p_list(i_p - 1,:) + [L_x,L_y] * rot_theta;
        diff_next = p_next - p_list(i_p-1,:);
        angle_next = rad2deg(atan2(diff_next(2),diff_next(1)));
        angle_change = angle_next - angle_list(i_p-1);
        if abs(angle_next - angle_list(i_p-1)) < 90
            bad_angle = 0;
            fprintf('Good angle %f\n', angle_next);
        else
            fprintf('No, I do not like that one\n')
        end       
    end
    angle_list(i_p) = angle_next;
    p_list(i_p,:) = p_next;
    fprintf('Adding next segment...\n')
    plot(p_list(1:i_p,1), p_list(1:i_p,2),'*-');
    %axis([min(min(p_list)),max(max(p_list)),min(min(p_list)),max(max(p_list))]);
    %pause;
end

angle_list = [angle_list(2:end);0];

end

