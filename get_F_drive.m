function Fk = get_F_drive(t,q,dq,ddq,F,r,L) %没用到
n = numel(q)/6 + 1;

% DH_Parameter = set_DH_Parameter(q,L);
g = [0;-9.8;0];

Mass = zeros(6*(n-1),6*(n-1));
Force = zeros(6*(n-1),1);
Body(1).Joint = set_Joint(zeros(6,1),zeros(6,1),1);
Body(1).T_qe_q = zeros(6,numel(q));
for i = 2:n
	qe = q([6*(i-2)+1:6*(i-2)+6]);
	dqe = dq([6*(i-2)+1:6*(i-2)+6]);
	Fe = F([6*(i-2)+1:6*(i-2)+6]);
	
	re = r(i-1);
	Le = L(i-1);
	
	[Body(i).Mass,Body(i).Force] = Mass_Force_element(re,Le,qe,dqe,Fe,g);
	
	Mass([6*(i-2)+1:6*(i-2)+6],[6*(i-2)+1:6*(i-2)+6]) = Body(i).Mass;
	Force([6*(i-2)+1:6*(i-2)+6],1) = Body(i).Force;
	
	Body(i).T_qe_q = zeros(6,numel(q));
	Body(i).T_qe_q(:,[6*(i-2)+1:6*(i-2)+6]) = eye(6);
	Body(i).Joint = set_Joint(qe,dqe,i);
	
% 	norm(Body(i).Joint(1).phi-Body(i).Joint(2).phi)
end
[g,B,dg,Tau] = add_Constraint(q,dq,Body);
Force = add_Hydraulic_Effect(q,dq,Force,Body);
alpha = 100;beta = 1;P = 0*ones(numel(g));
lambda = inv(B * inv(Mass) * B') * (Tau + 2 * alpha * beta * dg + alpha ^ 2 * g - B * inv(Mass) * (Force + B' * P * g));
ddq = inv(Mass) * (-Force - B'*lambda - B' * P * g);
T_qe_q = [Body(3).T_qe_q;Body(4).T_qe_q;Body(8).T_qe_q;Body(9).T_qe_q];
Fk = inv((eye(6*(n-1)) - B'*inv(B*inv(Mass)*B')*B*inv(Mass))) * ...
 	(Mass * ddq + Force + B'*inv(B*inv(Mass)*B')*(Tau+2*alpha*beta+alpha^2*g-B*inv(Mass)*Force));

Fk_Lambda = inv([-eye(48),B';-B*inv(Mass),B*inv(Mass)*B']) * ...
	[-(Mass*ddq+Force+B'*P*g);
	-(B*inv(Mass)*Force+B*inv(Mass)*B'*P*g)+(Tau+2*alpha*beta*dg+alpha^2*g)];


end