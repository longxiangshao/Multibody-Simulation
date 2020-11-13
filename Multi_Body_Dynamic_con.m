function dx = Multi_Body_Dynamic_con(t,x,u,F,r,L,number_AE0,number_LJ,m) %ode方程
q = x(1:numel(x)/2); %6nx1
dq = x(numel(x)/2+1:end); %6nx1
n = numel(q)/6 + 1;  %共10根杆件，算上基架

g = [0;-9.8;0];

Mass = zeros(6*(n-1),6*(n-1));%6nx6n 初始化质量阵
Force = zeros(6*(n-1),1);     %初始化力阵
Body(1).Joint = set_Joint(zeros(6,1),zeros(6,1),1,number_AE0,number_LJ); %底座所有相关点的参数
Body(1).T_qe_q = zeros(6,numel(q)); %（底座没有包含在状态变量）由entire dof得到底座dof；6x6n
for i = 2:n %2到11号杆件
	qe = q([6*(i-2)+1:6*(i-2)+6]); % 0_q_Q；6x1
	dqe = dq([6*(i-2)+1:6*(i-2)+6]); %0_dq_Q；6x1
	Fe = F([6*(i-2)+1:6*(i-2)+6]); %体积力，为0
	
	re = r(i-1); %i号杆件半径
	Le = L(i-1); %i号杆件长度
    me = m(i-1); %i号杆件质量
	
	[Body(i).Mass,Body(i).Force] = Mass_Force_element(re,Le,qe,dqe,Fe,g,me);%调用函数，获取body质量阵,力阵
	
	Mass([6*(i-2)+1:6*(i-2)+6],[6*(i-2)+1:6*(i-2)+6]) = Body(i).Mass;%转移到全局54x54质量阵
	Force([6*(i-2)+1:6*(i-2)+6],1) = Body(i).Force;%转移到全局54x1力阵
	
	Body(i).T_qe_q = zeros(6,numel(q));
	Body(i).T_qe_q(:,[6*(i-2)+1:6*(i-2)+6]) = eye(6); %6x54；由entire dof得到i系dof
	Body(i).Joint = set_Joint(qe,dqe,i,number_AE0,number_LJ);%设定每一个杆件上的Joint
	
end

[g,B,dg,Tau] = add_Constraint(q,dq,Body,number_AE0,number_LJ);%添加约束


%add_PDcontroller(t,q,dq);%控制液压缸的驱动力
%global u;
[u] = add_PDcontroller(t,u,q,dq);%控制液压缸的驱动力


number = n - 1;
[~,B_drive] = add_Driving(u,Force,q,r,Body,number); %添加驱动力
[F_TorSpr] = add_TorsionSpring(q,dq,Body,number_AE0,number_LJ); %torsion spring
Force = add_Hydraulic_Effect(q,dq,Force,Body);%添加液压效应；不重要

%多体动力学求解
alpha = 100;beta = 1;P = 0*ones(numel(g));%约束补充系数%P惩罚系数为0
lambda = (B * inv(Mass) * B')\(Tau + 2 * alpha * beta * dg + alpha ^ 2 * g - B * inv(Mass) * (Force - F_TorSpr - B_drive * u + B' * P * g));
%lamada中有一项也含u

phy = F_TorSpr - Force - B'*lambda - B' * P * g;
ddq = Mass \(phy + B_drive * u); %需要的动力学方程表达形式
% B_drive为确定控制力位置的矩阵

dq_dt = get_dqdt(q,dq);  %J(qe)与qe关系
dx = [dq_dt;ddq];  %构建状态方程

t
% pause(0.01);
end