function [output] = intersecting_lines(x1_start,x1_end,y1_start,y1_end,x2_start,x2_end,y2_start,y2_end)
% This functions takes as inputs 2 lines and returns whether they intersect or not
% boolean = intersecting_lines(x1_start,x1_end,y1_start,y1_end,x2_start,x2_end,y2_start,y2_end)
line_1 = [x1_start y1_start; x1_end y1_end]; % defines ship's perpendicular
line_2 = [x2_start y2_start; x2_end y2_end]; % defines pipe segment
d1 = diff(line_1);  % Take the differences down each column
d2 = diff(line_2);
den = d1(1)*d2(2)-d2(1)*d1(2);  % Precompute the denominator
ua = (d1(2)*(line_2(1)-line_2(3))-d2(2)*(line_1(1)-line_1(3)))/den;
ub = (d1(1)*(line_2(1)-line_2(3))-d2(1)*(line_1(1)-line_1(3)))/den; % should be same as ua, just for checking purposes
% Calculate the intersection point
xi = line_1(1)+ua*d1(1);
yi = line_2(1)+ua*d2(1);
% Intersection point
int_point = [xi yi];
% Ship position
ship_pos = [(x1_start + x2_start)/2; (y1_start + y2_start)/2]; 
% Distance from ship to intersection
dis = sqrt((int_point(1) - ship_pos(1))^2 + (int_point(2) - ship_pos(2))^2);

fprintf('Distance from ship to intersection point %f\n', dis);
fprintf('Intersection point [%f,%f]\n',int_point(1),int_point(2));
plot([x1_start,x1_end],[y1_start,y1_end]);
hold on; 
plot([x2_start,x2_end],[y2_start,y2_end]);
plot(int_point(1),int_point(2),'o');
if (xi >= x2_start) && (xi <= x2_end) && (dis <= 60)
    output = true;
else

    output = false;
end

