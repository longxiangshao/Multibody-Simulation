function Joint = set_Joint(q,dq,i,number_AE0,number_LJ)
%设置AE0杆的迭代部分
if i >=12 && i <= (11+number_AE0)
   i = 12; 
end
%设置LJ杆的迭代部分
if i >= (11+number_AE0+1) && i <= (11+number_AE0+number_LJ)
    i = 13;
end

switch i
	case 1 %1号底座相关各点
		r = [0;0;0]; %原点
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [-866;1500;0]; %与2号杆件交点；A0点
		Joint(2) = set_Joit_Parameter(r,q,dq);
		
		r = [866;500;0]; %与4号杆件交点；B0点
		Joint(3) = set_Joit_Parameter(r,q,dq);
		
		%plot Joint %画出基架三角形
		r = [-1732;2000;0];
		Joint(4) = set_Joit_Parameter(r,q,dq);
		
		r = [-1732;0;0];
		Joint(5) = set_Joit_Parameter(r,q,dq);
		
		r = [1732;0;0];
		Joint(6) = set_Joit_Parameter(r,q,dq);
	case 2		%2号杆件节点
		r = [0;0;0]; %2_r_A0
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [5000;0;0]; %2_r_A
		Joint(2) = set_Joit_Parameter(r,q,dq);
        
% 		r = [length_under;0;0]; %2_r_I
% 		Joint(3) = set_Joit_Parameter(r,q,dq);
	case 3    %3号杆件节点
		r = [0;0;0]; %3_r_A
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0]; 
		Joint(2) = set_Joit_Parameter(r,q,dq);	
	case 4    %4号杆件节点
		r = [0;0;0]; 
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0]; 
		Joint(2) = set_Joit_Parameter(r,q,dq);	
	case 5    %5号杆件节点
		r = [0;0;0];%D
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [5500;0;0]; %5_r_L
		Joint(2) = set_Joit_Parameter(r,q,dq);
        
        r = [500;-1000;0]; %5_r_F
		Joint(3) = set_Joit_Parameter(r,q,dq);
        
        r = [500;0;0]; %5_r_F0
		Joint(4) = set_Joit_Parameter(r,q,dq);
	case 6   %6号杆件节点
		r = [0;0;0]; %F
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [2000;0;0]; %6_r_H
		Joint(2) = set_Joit_Parameter(r,q,dq);
	case 7   %7号杆件节点
		r = [0;0;0];%E
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0]; %H
		Joint(2) = set_Joit_Parameter(r,q,dq);
	case 8   %8号杆件节点
		r = [0;0;0];
		Joint(1) = set_Joit_Parameter(r,q,dq);
		
		r = [1500;0;0];
		Joint(2) = set_Joit_Parameter(r,q,dq);
	case 9    %9号杆件节点
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

function Joint = set_Joit_Parameter(r,q,dq) %注意节点与目标点的区别
phi = q(4:6);
omega = dq(4:6);
R = get_R(phi);
 % 各目标点中包含的参数 
Joint.r = q(1:3) + R * r; %0_r_目标点
Joint.phi = phi; %目标点旋转角度
% Joint.dr = dq(1:3)-skew(R*r)*dq(4:6);
Joint.T_qi_q = eye(6);
Joint.T_qi_q(1:3,:) = [eye(3),-skew(R*r)];  %由0_v_Q求目标点在广义坐标系里的速度
Joint.dr = Joint.T_qi_q(1:3,:) * dq; %0_v_目标点
Joint.omega = omega; %w_目标点

end
