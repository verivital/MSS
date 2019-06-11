function [m2,n2] = perpendicular(x,y,angle)
%len = 60; % 0 meters each side to the ship
% Typical line express as y = mx + n
m = tan(angle); % slope of ship axis line
%n = y - m*x; % bias of ship axis line
% Now, find its perpendicular
m2 = -1/m; % slope of perpendicular line
n2 = y-m2*x; % bias of perpendicular line
end

