function dq_dt = get_dqdt(q,dq)

n = numel(q) / 6;
dq_dt = zeros(numel(q),1);
for i = 1:n
	dq_dt([6*(i-1)+1:6*(i-1)+3]) = dq([6*(i-1)+1:6*(i-1)+3]);
	
	phi = q(6*(i-1)+4:6*(i-1)+6);
	omega = dq(6*(i-1)+4:6*(i-1)+6);
	T = get_T(phi);
%   T=eye(3);
	dq_dt([6*(i-1)+4:6*(i-1)+6]) = T \ omega;
end

end