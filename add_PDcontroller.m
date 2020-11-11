function [u] = add_PDcontroller(t,u,q,dq)
%����Һѹ�׵�������
kp = 10000;kd = 100;
F_AB0 = u(2);F_LH = u(3);
%��Һѹ��
L0_AB0 = 3000; %A��B0֮������ľ���[mm]
x_A = q(7);y_A = q(8);x_B0 = 866;y_B0 = 500;
L_AB0 = CalDistance(x_A,y_A,x_B0,y_B0); %����ʵ�ʾ���
s_AB0 = L_AB0 - L0_AB0;
v_A = sqrt(dq(7)^2 + dq(8)^2);%����A���ʵ���ٶ�
v_AB0 = v_A; 
%disp(L_AB0);
t1_AB0 = 1;t2_AB0 = 8;t3_AB0 = 1;v0_AB0 = 128;dflag_AB0 = 0; %t1:���� t2:���� t3:���� v0:Һѹ�������ٶ�
[v_soll_AB0,s_soll_AB0] = velocity_function_2(t,t1_AB0,t2_AB0,t3_AB0,v0_AB0,dflag_AB0);%����Ŀ��ֵ
e_AB0 = s_soll_AB0 - s_AB0;
e_d_AB0 = v_soll_AB0 - v_AB0;

u(2) = F_AB0 + kp*e_AB0 + kd*e_d_AB0;
disp(u(2)*10^-6);

%��Һѹ��
L0_LH = 2570.992;
x_L = q(43);y_L = q(44);x_H = q(37);y_H = q(38);
L_LH = CalDistance(x_L,y_L,x_H,y_H);
s_LH = L_LH - L0_LH;
t1_LH = 2;t2_LH = 8;t3_LH = 2;v0_LH = 128;dflag_LH = 0;
[v_soll_LH,s_soll_LH] = velocity_function_2(t,t1_LH,t2_LH,t3_LH,v0_LH,dflag_LH);
e_LH = s_soll_LH - s_LH;
v_L = sqrt(dq(43)^2 + dq(44)^2);
v_H = sqrt(dq(37)^2 + dq(38)^2);
v_LH = v_L - v_H;
e_d_LH = v_soll_LH - v_LH;

u(3) = F_LH + kp*e_LH + kd*e_d_LH;
%disp(s_LH)
disp(u(3)*10^-6)
end

