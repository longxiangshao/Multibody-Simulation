function [output] = CalAngle(x1,y1,x2,y2)
angle = atan2((y2-y1),(x2-x1));
output = rad2deg(angle);
end
