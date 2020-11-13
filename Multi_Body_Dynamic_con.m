function dx = Multi_Body_Dynamic_con(t,x,u,F,r,L,number_AE0,number_LJ,m) %ode����
q = x(1:numel(x)/2); %6nx1
dq = x(numel(x)/2+1:end); %6nx1
n = numel(q)/6 + 1;  %��10���˼������ϻ���

g = [0;-9.8;0];

Mass = zeros(6*(n-1),6*(n-1));%6nx6n ��ʼ��������
Force = zeros(6*(n-1),1);     %��ʼ������
Body(1).Joint = set_Joint(zeros(6,1),zeros(6,1),1,number_AE0,number_LJ); %����������ص�Ĳ���
Body(1).T_qe_q = zeros(6,numel(q)); %������û�а�����״̬��������entire dof�õ�����dof��6x6n
for i = 2:n %2��11�Ÿ˼�
	qe = q([6*(i-2)+1:6*(i-2)+6]); % 0_q_Q��6x1
	dqe = dq([6*(i-2)+1:6*(i-2)+6]); %0_dq_Q��6x1
	Fe = F([6*(i-2)+1:6*(i-2)+6]); %�������Ϊ0
	
	re = r(i-1); %i�Ÿ˼��뾶
	Le = L(i-1); %i�Ÿ˼�����
    me = m(i-1); %i�Ÿ˼�����
	
	[Body(i).Mass,Body(i).Force] = Mass_Force_element(re,Le,qe,dqe,Fe,g,me);%���ú�������ȡbody������,����
	
	Mass([6*(i-2)+1:6*(i-2)+6],[6*(i-2)+1:6*(i-2)+6]) = Body(i).Mass;%ת�Ƶ�ȫ��54x54������
	Force([6*(i-2)+1:6*(i-2)+6],1) = Body(i).Force;%ת�Ƶ�ȫ��54x1����
	
	Body(i).T_qe_q = zeros(6,numel(q));
	Body(i).T_qe_q(:,[6*(i-2)+1:6*(i-2)+6]) = eye(6); %6x54����entire dof�õ�iϵdof
	Body(i).Joint = set_Joint(qe,dqe,i,number_AE0,number_LJ);%�趨ÿһ���˼��ϵ�Joint
	
end

[g,B,dg,Tau] = add_Constraint(q,dq,Body,number_AE0,number_LJ);%���Լ��


%add_PDcontroller(t,q,dq);%����Һѹ�׵�������
%global u;
[u] = add_PDcontroller(t,u,q,dq);%����Һѹ�׵�������


number = n - 1;
[~,B_drive] = add_Driving(u,Force,q,r,Body,number); %���������
[F_TorSpr] = add_TorsionSpring(q,dq,Body,number_AE0,number_LJ); %torsion spring
Force = add_Hydraulic_Effect(q,dq,Force,Body);%���ҺѹЧӦ������Ҫ

%���嶯��ѧ���
alpha = 100;beta = 1;P = 0*ones(numel(g));%Լ������ϵ��%P�ͷ�ϵ��Ϊ0
lambda = (B * inv(Mass) * B')\(Tau + 2 * alpha * beta * dg + alpha ^ 2 * g - B * inv(Mass) * (Force - F_TorSpr - B_drive * u + B' * P * g));
%lamada����һ��Ҳ��u

phy = F_TorSpr - Force - B'*lambda - B' * P * g;
ddq = Mass \(phy + B_drive * u); %��Ҫ�Ķ���ѧ���̱����ʽ
% B_driveΪȷ��������λ�õľ���

dq_dt = get_dqdt(q,dq);  %J(qe)��qe��ϵ
dx = [dq_dt;ddq];  %����״̬����

t
% pause(0.01);
end