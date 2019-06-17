function [output] = intersecting_lines(L,x1_start,x1_end,y1_start,y1_end,x2_start,x2_end,y2_start,y2_end)
% This functions takes as inputs 2 lines and returns whether they intersect or not
% It "simulates" a side sonar 
% boolean = intersecting_lines(x1_start,x1_end,y1_start,y1_end,x2_start,x2_end,y2_start,y2_end)

%line_1 = [x1_start y1_start; x1_end y1_end]; % defines ship's perpendicular
%line_2 = [x2_start y2_start; x2_end y2_end]; % defines pipe segment

% Precompute the denominator
den = (y2_end - y2_start)*(x1_end - x1_start) - (x2_end - x2_start)*(y1_end - y1_start);
ua = ((x2_end - x2_start)*(y1_start - y2_start) - (y2_end - y2_start)*(x1_start - x2_start))/den;

% Calculate the intersection point coordinates
xi = x1_start + ua*(x1_end-x1_start);
yi = y1_start + ua*(y1_end-y1_start);
% Intersection point
int_point = [xi yi];

% Ship position
ship_pos = [(x1_start + x1_end)/2; (y1_start + y1_end)/2]; 

% Distance from ship to intersection
dis = sqrt((int_point(1) - ship_pos(1))^2 + (int_point(2) - ship_pos(2))^2);

% Calculate side sonar range
if L < 5 
   sonar_range = 60;
elseif L < 30
    sonar_range = 200;
elseif L < 120 
    sonar_range = 400;
else
    sonar_range = 600;
end

%fprintf('Distance from ship to intersection point %f\n', dis);
%fprintf('Intersection point [%f,%f]\n',int_point(1),int_point(2));
%plot([x1_start,x1_end],[y1_start,y1_end],'LineWidth',0.01);
%hold on; 
%plot([x2_start,x2_end],[y2_start,y2_end]);
%plot(int_point(1),int_point(2),'o');
plot(ship_pos(1),ship_pos(2),'s');
if (xi >= x2_start) && (xi <= x2_end) && (dis <= sonar_range)
    output = true;
else

    output = false;
end

end