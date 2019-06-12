function [x1,x2,y1,y2] = perpendicular_to_ship(x,y,angle)
len = 60; % 0 meters each side to the ship

% Typical line express as y = mx + n (one way of expressing it)
% m = tan(angle); % slope of ship axis line
% n = y - m*x; % bias of ship axis line
% Now, find its perpendicular
% m2 = -1/m; % slope of perpendicular line
% n2 = y-m2*x; % bias of perpendicular line

% Second way of expressing line segments
% Calculate end points of perpendicular line up to a distance of 60 meters 
% to each side (range of side sonar)
angle = angle + pi/2;
x1 = x-len*cos(angle);
x2 = x+len*cos(angle);
y1 = y-len*sin(angle);
y2 = y+len*sin(angle);
fprintf('Perpendicular to ship axis: (x1,x2,y1,y2) [%f,%f,%f,%f]\n',x1,x2,y1,y2);
end

