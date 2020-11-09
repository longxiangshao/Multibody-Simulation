function [g,B,dg,Tau] = set_constraint(q,dq,Bodyi,i,Bodyj,j,type) %g:constraint
%% 定义两个body间计算所需参数
Joint_i = Bodyi.Joint(i);
qi = [Joint_i.r;Joint_i.phi];%bodyi:0_q_目标点
dqi = [Joint_i.dr;Joint_i.omega]; %bodyi:0_v_目标点
T_qi_q = Joint_i.T_qi_q * Bodyi.T_qe_q; %6x54；由entire dof的速度求目标点所在的reference point的速度，再求目标点的速度
Ri = get_R(qi(4:6)); %旋转矩阵
xi = Ri(:,1);yi = Ri(:,2);zi = Ri(:,3); %xyz轴
r0i = Joint_i.r; %位置
omega_i = dqi(4:6);%角速度

Joint_j = Bodyj.Joint(j); %body j-第j个节点变量设置
qj = [Joint_j.r;Joint_j.phi];
dqj = [Joint_j.dr;Joint_j.omega];
T_qj_q = Joint_j.T_qi_q * Bodyj.T_qe_q;
Rj = get_R(qj(4:6));
xj = Rj(:,1);yj = Rj(:,2);zj = Rj(:,3);
r0j = Joint_j.r;
omega_j = dqj(4:6);
%% 
switch type          
	case 'Revolute'  %平面铰链约束
		g = zeros(5,1);
		g(1:3) = r0i - r0j;
        g(4) = yi' * zj;
        g(5) = xi' * zj;

        %B为约束力方向
		B = zeros(5,numel(q)); %5x54
        B([1:3],:) = [eye(3),zeros(3)]*T_qi_q - [eye(3),zeros(3)]*T_qj_q; %用T是因为有的关节点不是body的reference point
		B(4,:) = [zeros(1,3),-(zj)'*skew(yi)] * T_qi_q + ...
			[zeros(1,3),-(yi)'*skew(zj)] * T_qj_q;
		B(5,:) = [zeros(1,3),-(zj)'*skew(xi)] * T_qi_q + ...
			[zeros(1,3),-(xi)'*skew(zj)] * T_qj_q;
		
		dg = B*dq;%约束对时间一阶导数,用于求lambda
		%约束对时间二阶导数,用于Baumgarte stabilization
		Tau = zeros(5,1);
		Tau([1:3],:) = [zeros(3,3),-skew(omega_i)*skew(r0i)] * Bodyi.T_qe_q * dq - ...
			[zeros(3,3),-skew(omega_j)*skew(r0j)] * Bodyj.T_qe_q * dq;
		Tau(4,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(yi)-zj'*skew(omega_i)*skew(yi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*yi)'*skew(zj)-yi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
		Tau(5,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(xi)-zj'*skew(omega_i)*skew(xi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*xi)'*skew(zj)-xi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
        
	case 'Prismatic' %平面滑动约束---对应液压系统
		g = zeros(5,1);
		g(1) = yi' * (r0i - r0j); %位移约束
		g(2) = zi' * (r0i - r0j);
		g(3) = yi' * zj; %转角约束
		g(4) = xi' * zj;
		g(5) = xi' * yj;
		
		B = zeros(5,numel(q)); 
		B(1,:) = [yi',-(r0i - r0j)'*skew(yi)] * T_qi_q + ...
			[-yi',zeros(1,3)] * T_qj_q;
		B(2,:) = [zi',-(r0i - r0j)'*skew(zi)] * T_qi_q + ...
			[-zi',zeros(1,3)] * T_qj_q;
		B(3,:) = [zeros(1,3),-(zj)'*skew(xi)] * T_qi_q + ...
			[zeros(1,3),-(xi)'*skew(zj)] * T_qj_q;
		B(4,:) = [zeros(1,3),-(zj)'*skew(yi)] * T_qi_q + ...
			[zeros(1,3),-(yi)'*skew(zj)] * T_qj_q;
		B(5,:) = [zeros(1,3),-(yj)'*skew(xi)] * T_qi_q + ...
			[zeros(1,3),-(xi)'*skew(yj)] * T_qj_q;
		
		dg = B*dq;%平面滑动约束对时间的一次导数
		
		Tau = zeros(5,1);%平面滑动约束对时间的二阶导数
		Tau(1,:) = -2*(dqi(1:3)-dqj(1:3))'*skew(yi)*omega_i - (r0i-r0j)'*skew(omega_i)*skew(yi)*omega_i;
		Tau(2,:) = -2*(dqi(1:3)-dqj(1:3))'*skew(zi)*omega_i - (r0i-r0j)'*skew(omega_i)*skew(zi)*omega_i;
		Tau(3,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(xi)-zj'*skew(omega_i)*skew(xi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*xi)'*skew(zj)-xi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
		Tau(4,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(yi)-zj'*skew(omega_i)*skew(yi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*yi)'*skew(zj)-yi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
		Tau(5,:) = [zeros(1,3),-(skew(omega_j)*yj)'*skew(xi)-yj'*skew(omega_i)*skew(xi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*xi)'*skew(yj)-xi'*skew(omega_j)*skew(yj)] * T_qj_q * dq;
        
        case 'Fixed' %关节固定的情况
		g = zeros(6,1);
		g(1:3) = r0i - r0j;
		g(4) = xi' * zj;
		g(5) = yi' * zj;
		g(6) = xi' * yj;
		
		B = zeros(6,numel(q));
		B([1:3],:) = T_qi_q(1:3,:) - T_qj_q(1:3,:);
		B(4,:) = [zeros(1,3),-(zj)'*skew(xi)] * T_qi_q + ...
			[zeros(1,3),-(xi)'*skew(zj)] * T_qj_q;
		B(5,:) = [zeros(1,3),-(zj)'*skew(yi)] * T_qi_q + ...
			[zeros(1,3),-(yi)'*skew(zj)] * T_qj_q;
		B(6,:) = [zeros(1,3),-(yj)'*skew(xi)] * T_qi_q + ...
			[zeros(1,3),-(xi)'*skew(yj)] * T_qj_q;
		
		dg = B*dq;
		
		Tau = zeros(6,1);
		Tau([1:3],:) = [zeros(3,3),-skew(omega_i)*skew(r0i)] * Bodyi.T_qe_q * dq - ...
			[zeros(3,3),-skew(omega_j)*skew(r0j)] * Bodyj.T_qe_q * dq;
		Tau(4,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(xi)-zj'*skew(omega_i)*skew(xi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*xi)'*skew(zj)-xi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
		Tau(5,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(yi)-zj'*skew(omega_i)*skew(yi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*yi)'*skew(zj)-yi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
		Tau(6,:) = [zeros(1,3),-(skew(omega_j)*yj)'*skew(xi)-yj'*skew(omega_i)*skew(xi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*xi)'*skew(yj)-xi'*skew(omega_j)*skew(yj)] * T_qj_q * dq;
		
	case 'Closed_Revolute' %四杆机构运动时--共18个自由度---但是约束20个自由度--系统冗余
		n = 2;  %四杆机构中最后一个铰链，约束由5个变为2个-----共17个约束---
		choose_matrix = eye(3);
		choose_matrix = choose_matrix(1:n,:);
		
		g = zeros(2,1);
		g(1:2) = choose_matrix * (r0i - r0j); %约束的只有最后节点位移的前两个值--z方向不用考虑
		
		B = zeros(2,numel(q));%如约束冗余，B会出现非满秩矩阵
		B([1:2],:) = choose_matrix * (T_qi_q(1:3,:) - T_qj_q(1:3,:));
		
		dg = B*dq;
		
		Tau = zeros(2,1);
		Tau([1:2],:) = choose_matrix * ([zeros(3,3),-skew(omega_i)*skew(r0i)] * Bodyi.T_qe_q * dq - ...
			[zeros(3,3),-skew(omega_j)*skew(r0j)] * Bodyj.T_qe_q * dq);
end

end