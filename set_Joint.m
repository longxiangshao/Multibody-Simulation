function Joint = set_Joint(q,dq,i,number_AE0,number_LJ)
%����AE0�˵ĵ�������
if i >=12 && i <= (11+number_AE0)
   i = 12; 
end
%����LJ�˵ĵ�������
if i >= (11+number_AE0+1) && i <= (11+number_AE0+number_LJ)
    i = 13;
end

switch i
	case 1 %1�ŵ�����ظ���
		r = [0;0;0]; %ԭ��
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [-866;1500;0]; %��2�Ÿ˼����㣻A0��
		Joint(2) = set_Joit_Parameter(r,q,dq);
		
		r = [866;500;0]; %��4�Ÿ˼����㣻B0��
		Joint(3) = set_Joit_Parameter(r,q,dq);
		
		%plot Joint %��������������
		r = [-1732;2000;0];
		Joint(4) = set_Joit_Parameter(r,q,dq);
		
		r = [-1732;0;0];
		Joint(5) = set_Joit_Parameter(r,q,dq);
		
		r = [1732;0;0];
		Joint(6) = set_Joit_Parameter(r,q,dq);
	case 2		%2�Ÿ˼��ڵ�
		r = [0;0;0]; %2_r_A0
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [5000;0;0]; %2_r_A
		Joint(2) = set_Joit_Parameter(r,q,dq);
        
% 		r = [length_under;0;0]; %2_r_I
% 		Joint(3) = set_Joit_Parameter(r,q,dq);
	case 3    %3�Ÿ˼��ڵ�
		r = [0;0;0]; %3_r_A
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0]; 
		Joint(2) = set_Joit_Parameter(r,q,dq);	
	case 4    %4�Ÿ˼��ڵ�
		r = [0;0;0]; 
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0]; 
		Joint(2) = set_Joit_Parameter(r,q,dq);	
	case 5    %5�Ÿ˼��ڵ�
		r = [0;0;0];%D
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [5500;0;0]; %5_r_L
		Joint(2) = set_Joit_Parameter(r,q,dq);
        
        r = [500;-1000;0]; %5_r_F
		Joint(3) = set_Joit_Parameter(r,q,dq);
        
        r = [500;0;0]; %5_r_F0
		Joint(4) = set_Joit_Parameter(r,q,dq);
	case 6   %6�Ÿ˼��ڵ�
		r = [0;0;0]; %F
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [2000;0;0]; %6_r_H
		Joint(2) = set_Joit_Parameter(r,q,dq);
	case 7   %7�Ÿ˼��ڵ�
		r = [0;0;0];%E
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0]; %H
		Joint(2) = set_Joit_Parameter(r,q,dq);
	case 8   %8�Ÿ˼��ڵ�
		r = [0;0;0];
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0];
		Joint(2) = set_Joit_Parameter(r,q,dq);
	case 9    %9�Ÿ˼��ڵ�
		r = [0;0;0];
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0];
		Joint(2) = set_Joit_Parameter(r,q,dq);	
    case 10
        r = [0;0;0]; 
        Joint(1) = set_Joit_Parameter(r,q,dq);
        
        r = [500 + 7500/(number_AE0+1);0;0]; %D
        Joint(2) = set_Joit_Parameter(r,q,dq);
        
        r = [7500/(number_AE0+1);-1000;0]; %E
        Joint(3) = set_Joit_Parameter(r,q,dq);   
        
        r = [7500/(number_AE0+1);0;0]; %E0
        Joint(4) = set_Joit_Parameter(r,q,dq);        
    case 11
        r = [0;0;0];%L
        Joint(1) = set_Joit_Parameter(r,q,dq);
        
        r = [7500/(number_LJ+1);0;0];
        Joint(2) = set_Joit_Parameter(r,q,dq);
    case 12
        r = [0;0;0];
        Joint(1) = set_Joit_Parameter(r,q,dq);
        
        r = [7500/(number_AE0+1);0;0];
        Joint(2) = set_Joit_Parameter(r,q,dq);
    case 13
        r = [0;0;0];
        Joint(1) = set_Joit_Parameter(r,q,dq);
        
        r = [7500/(number_LJ+1);0;0];
        Joint(2) = set_Joit_Parameter(r,q,dq);
end

end

function Joint = set_Joit_Parameter(r,q,dq) %ע��ڵ���Ŀ��������
phi = q(4:6);
omega = dq(4:6);
R = get_R(phi);
 % ��Ŀ����а����Ĳ��� 
Joint.r = q(1:3) + R * r; %0_r_Ŀ���
Joint.phi = phi; %Ŀ�����ת�Ƕ�
% Joint.dr = dq(1:3)-skew(R*r)*dq(4:6);
Joint.T_qi_q = eye(6);
Joint.T_qi_q(1:3,:) = [eye(3),-skew(R*r)];  %��0_v_Q��Ŀ����ڹ�������ϵ����ٶ�
Joint.dr = Joint.T_qi_q(1:3,:) * dq; %0_v_Ŀ���
Joint.omega = omega; %w_Ŀ���

end
