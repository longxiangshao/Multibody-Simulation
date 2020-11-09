function [g,B,dg,Tau] = set_constraint(q,dq,Bodyi,i,Bodyj,j,type) %g:constraint
%% ��������body������������
Joint_i = Bodyi.Joint(i);
qi = [Joint_i.r;Joint_i.phi];%bodyi:0_q_Ŀ���
dqi = [Joint_i.dr;Joint_i.omega]; %bodyi:0_v_Ŀ���
T_qi_q = Joint_i.T_qi_q * Bodyi.T_qe_q; %6x54����entire dof���ٶ���Ŀ������ڵ�reference point���ٶȣ�����Ŀ�����ٶ�
Ri = get_R(qi(4:6)); %��ת����
xi = Ri(:,1);yi = Ri(:,2);zi = Ri(:,3); %xyz��
r0i = Joint_i.r; %λ��
omega_i = dqi(4:6);%���ٶ�

Joint_j = Bodyj.Joint(j); %body j-��j���ڵ��������
qj = [Joint_j.r;Joint_j.phi];
dqj = [Joint_j.dr;Joint_j.omega];
T_qj_q = Joint_j.T_qi_q * Bodyj.T_qe_q;
Rj = get_R(qj(4:6));
xj = Rj(:,1);yj = Rj(:,2);zj = Rj(:,3);
r0j = Joint_j.r;
omega_j = dqj(4:6);
%% 
switch type          
	case 'Revolute'  %ƽ�����Լ��
		g = zeros(5,1);
		g(1:3) = r0i - r0j;
        g(4) = yi' * zj;
        g(5) = xi' * zj;

        %BΪԼ��������
		B = zeros(5,numel(q)); %5x54
        B([1:3],:) = [eye(3),zeros(3)]*T_qi_q - [eye(3),zeros(3)]*T_qj_q; %��T����Ϊ�еĹؽڵ㲻��body��reference point
		B(4,:) = [zeros(1,3),-(zj)'*skew(yi)] * T_qi_q + ...
			[zeros(1,3),-(yi)'*skew(zj)] * T_qj_q;
		B(5,:) = [zeros(1,3),-(zj)'*skew(xi)] * T_qi_q + ...
			[zeros(1,3),-(xi)'*skew(zj)] * T_qj_q;
		
		dg = B*dq;%Լ����ʱ��һ�׵���,������lambda
		%Լ����ʱ����׵���,����Baumgarte stabilization
		Tau = zeros(5,1);
		Tau([1:3],:) = [zeros(3,3),-skew(omega_i)*skew(r0i)] * Bodyi.T_qe_q * dq - ...
			[zeros(3,3),-skew(omega_j)*skew(r0j)] * Bodyj.T_qe_q * dq;
		Tau(4,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(yi)-zj'*skew(omega_i)*skew(yi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*yi)'*skew(zj)-yi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
		Tau(5,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(xi)-zj'*skew(omega_i)*skew(xi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*xi)'*skew(zj)-xi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
        
	case 'Prismatic' %ƽ�滬��Լ��---��ӦҺѹϵͳ
		g = zeros(5,1);
		g(1) = yi' * (r0i - r0j); %λ��Լ��
		g(2) = zi' * (r0i - r0j);
		g(3) = yi' * zj; %ת��Լ��
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
		
		dg = B*dq;%ƽ�滬��Լ����ʱ���һ�ε���
		
		Tau = zeros(5,1);%ƽ�滬��Լ����ʱ��Ķ��׵���
		Tau(1,:) = -2*(dqi(1:3)-dqj(1:3))'*skew(yi)*omega_i - (r0i-r0j)'*skew(omega_i)*skew(yi)*omega_i;
		Tau(2,:) = -2*(dqi(1:3)-dqj(1:3))'*skew(zi)*omega_i - (r0i-r0j)'*skew(omega_i)*skew(zi)*omega_i;
		Tau(3,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(xi)-zj'*skew(omega_i)*skew(xi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*xi)'*skew(zj)-xi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
		Tau(4,:) = [zeros(1,3),-(skew(omega_j)*zj)'*skew(yi)-zj'*skew(omega_i)*skew(yi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*yi)'*skew(zj)-yi'*skew(omega_j)*skew(zj)] * T_qj_q * dq;
		Tau(5,:) = [zeros(1,3),-(skew(omega_j)*yj)'*skew(xi)-yj'*skew(omega_i)*skew(xi)] * T_qi_q * dq + ...
			[zeros(1,3),-(skew(omega_i)*xi)'*skew(yj)-xi'*skew(omega_j)*skew(yj)] * T_qj_q * dq;
        
        case 'Fixed' %�ؽڹ̶������
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
		
	case 'Closed_Revolute' %�ĸ˻����˶�ʱ--��18�����ɶ�---����Լ��20�����ɶ�--ϵͳ����
		n = 2;  %�ĸ˻��������һ��������Լ����5����Ϊ2��-----��17��Լ��---
		choose_matrix = eye(3);
		choose_matrix = choose_matrix(1:n,:);
		
		g = zeros(2,1);
		g(1:2) = choose_matrix * (r0i - r0j); %Լ����ֻ�����ڵ�λ�Ƶ�ǰ����ֵ--z�����ÿ���
		
		B = zeros(2,numel(q));%��Լ�����࣬B����ַ����Ⱦ���
		B([1:2],:) = choose_matrix * (T_qi_q(1:3,:) - T_qj_q(1:3,:));
		
		dg = B*dq;
		
		Tau = zeros(2,1);
		Tau([1:2],:) = choose_matrix * ([zeros(3,3),-skew(omega_i)*skew(r0i)] * Bodyi.T_qe_q * dq - ...
			[zeros(3,3),-skew(omega_j)*skew(r0j)] * Bodyj.T_qe_q * dq);
end

end