function T = get_T(phi) %Jacobian

theta = norm(phi);
if theta == 0
% 	T = eye(3) + 1 / 2 * skew(phi) + 1 / 6 * skew(phi) * skew(phi);
    T=eye(3);
else
% 	T = eye(3) + (1 - cos(theta)) / (theta ^ 2) * skew(phi) + (theta - sin(theta)) / (theta ^ 3) * skew(phi) * skew(phi);
    T = eye(3) + (cos(theta)-1)/(theta ^ 2) * skew(phi) + (1-sin(theta)/theta) * skew(phi)^2 / (theta^2);
end

end