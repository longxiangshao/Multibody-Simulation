function R = get_R(phi) %Cartesian rotation matrix������ֱ��ת������������ϵ

theta = norm(phi);
if theta == 0
	R = eye(3);
else
	R = eye(3)*cos(theta) + sin(theta) / theta * skew(phi) + (1 - cos(theta)) / (theta ^ 2) * phi * phi';
end

end