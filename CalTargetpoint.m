function [x2,y2] = CalTargetpoint(x1,y1,phi,L)
x2 = x1 + L * cos(deg2rad(phi));
y2 = y1 + L * sin(deg2rad(phi));
end

