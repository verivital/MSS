%% Generate pipe segments for the ships and UUVs to follow
clc;clear;close all;
% Range where pipeline begins
x_range = [-100,100];
y_range = [-25,100];

% Initial point of pipeline
x_init = x_range(1) + (x_range(2) - x_range(1)).*rand(1,1);
y_init = y_range(1) + (y_range(2) - y_range(1)).*rand(1,1);
p_init = [x_init,y_init];

% number of points (number of line segments = p_N - 1)
p_N = 50;

% max length of pipe (along axes, triangle may be larger)
% For actual UUV
% L_max = 500;
% L_min = 25; 
% For mariner
% L_max = 3000;
% L_min = 1500; 
L_hyp = sqrt(L_max^2 + L_min^2);

% List of begin-end points for pipes
p_list = zeros(p_N,2);
p_list(1,:) = p_init;

% List of angles
angle_list = zeros(p_N-1,1);
%angle_list(1,:) = 0; % degrees

% Angle limits 
theta_min = deg2rad(-180);
theta_max = deg2rad(180);
%theta_dot_max = deg2rad(90);

any_bad = 0; % check if bad angles are generated (not allowed in simulation)

opt_intersect = 0; % check if segments intersect

%figure; hold on;

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
%         if opt_intersect
%             for i_theta = 1 : i_p - 1
%                 %if norm(p_next - p_list(i_theta,:),2) <= L_hyp
%                     % check intersection of nearby line segments
%                     %ls1 = polyshape(p_next, p_list(i_theta,:) + p_next);
%                     %ls1
% 
%                     for i_int = 2 : i_p - 1
%                         %ls2 = polyshape(p_list(i_int - 1,: ), p_list(i_int - 1, :) + p_list(i_int,:) );
% 
%                         %ls_int = intersect(ls1,ls2);
%                         ls_int = polyxpoly( p_next, p_list(i_theta,:) , p_list(i_int - 1,: ), p_list(i_int,:) );
% 
%                         %plot(p_next, p_list(i_theta,:) , p_list(i_int - 1,: ), p_list(i_int,:));
% 
%                         if ~isempty(ls_int)
%                             any_bad = 1;
%                             break;
%                         else
%                             bad_angle = 0;
%                         end
%                     end
% 
%                     if any_bad
%                         bad_angle = 1;
%                         any_bad = 0;
%                         break;
%                     end
%                 %end
%                 %if norm(p_next - p_list(i_theta,:),2) < L_hyp
%                 %    bad_angle = bad_angle && 0;
%                 %    break;
%                 %end
%             end
%         else
%             bad_angle = 0
%         end
    end
    angle_list(i_p) = angle_next;
    p_list(i_p,:) = p_next;
    fprintf('Adding next segment...\n')
    plot(p_list(1:i_p,1), p_list(1:i_p,2),'*-');
    axis([min(min(p_list)),max(max(p_list)),min(min(p_list)),max(max(p_list))]);
    %pause;
end

angle_list = [angle_list(2:end);0];

%plot(p_list(:,1), p_list(:,2),'*-');
% mm = max(max(abs(p_list))); 
% axis([-mm mm -mm mm]) % Fix axis dimensions so plot figure is square