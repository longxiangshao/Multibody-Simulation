clear;clc;
format long; %���������д�����ʾ��ʽ
opt=odeset('RelTol',0.1,'AbsTol',5,'MaxStep',5);
number_AE0 = 10;%�������Ǹ�����AE0���ּ��뼸���ؽ�
number_LJ = 10;%�������Ǹ�����FL���ּ��뼸���ؽ�
[t_set,x] = Test(opt,number_AE0,number_LJ);
plot_nodes_postprocessing(x,t_set,number_AE0,number_LJ);

function [t_set,x] = Test(opt,number_AE0,number_LJ)
n = 10 + number_AE0 + number_LJ; %�˼�����������������
L_DE0 = 300;
% L_DF0 = 300; L_EE0 = 600; L_FF0 = 300;

m = zeros(n,1);
m(1:10) = [456,10,10,400,5,5,10,10,1000,1000]; %[kg]
m_DE0 = 100 * L_DE0 * 0.001; m_AD = 1000;m_LJ = 1000;
m(9) = (m_AD - m_DE0)/(number_AE0 + 1)+ m_DE0; %10�Ÿ�����
m(11:10+number_AE0) = (m_AD - m_DE0)/(number_AE0 + 1); %AE0���سɵ�ÿС�ε�����

m(10) = m_LJ /(number_LJ + 1);
m(10+number_AE0+1:10+number_AE0+number_LJ) = m_LJ /(number_LJ + 1);%LJ���سɵ�ÿС�ε�����

r = zeros(n,1)+150;% r = [150,35,50,150,50,50,50,35,150,150,150...];�˼�Բ����뾶����λmm
r(2) = 35;r(3)=50;r(5)=50;r(6)=50;r(7)=50;r(8)=35;

L_part_AE0 = (10000-300) /(1 + number_AE0);%�����Ǹ�����AE0���ֱ��ؽڽضϳɵ�С�˵ĳ���
L_part_LJ = 10000 /(1 + number_LJ);%�����Ǹ�����LF���ֱ��ؽڽضϳɵ�С�˵ĳ���

L = zeros(n,1);
L_0 = [4560,3000,3000,4000,1500,1200,2559.79,2559.79,10000,10000];%ʮ���˳���
%�������泤�˵ĳ���
L(1:8)=L_0(1:8);
L(9)=L_DE0 + L_part_AE0; %AE0�ضϺ�ĵ�һ�����ǰ���DE0
L(10+1:10+number_AE0) = L_part_AE0;
%�������泤�˵ĳ���
L(10) = L_part_LJ;
L(10+number_AE0+1:10+number_AE0+number_LJ) = L_part_LJ;

q = zeros(6*n,1);
q(1:60) = [-866;1500;0;%0_r_2��A0   %�������� %���ڵ��ʼλ��(����������)
		0;0;deg2rad(0);%phi2������ڹ�������x�����ת�Ƕ�
		3694.4;1500;0;%0_r_3��A
		0;0;deg2rad(-160.53);%phi3
		866;500;0;%0_r_4��B0
		0;0;deg2rad(19.47);%phi4
		13694.4;1500;0;%0_r_5��D
		0;0;deg2rad(180);%phi5
		13394.4;1800;0;%0_r_6��F
		0;0;deg2rad(-143.13);%phi6
		13394.4;900;0;%0_r_7��E
		0;0;deg2rad(180);%phi7
		12194.4;900;0;%0_r_8��H
		0;0;deg2rad(166.5);%phi8
		9694.4;1500;0;%0_r_9��L
		0;0;deg2rad(-13.5);%phi9
        3694.4;1500;0;%0_r_10��A
        0;0;deg2rad(0);%phi10
        9694.4;1500;0;%0_r_11��L
        0;0;deg2rad(180);%phi11
		];
x_A = q(7);y_A = q(8);%A���ʼ����
x_E0 = 13394.4;y_E0 = 1500;%E0���ʼ����
x_part_AE0 = (x_E0 - x_A)/(number_AE0 + 1);%AE0���سɵ�С����x�����ϵľ��Գ���
y_part_AE0 = (y_E0 - y_A)/(number_AE0 + 1);%AE0���سɵ�С����y�����ϵľ��Գ���
%0_r_10����AE0���سɵ����һ�ε�����ϵλ��
q(49) = number_AE0 * x_part_AE0 + x_A;    
q(50) = number_AE0 * y_part_AE0 + y_A;
q(54) = deg2rad(0);
%�������һ���⣬AE0���سɵ���������С�εĳ�ʼ����ϵλ������
for number = 1:number_AE0
    q(60 + (number-1)*6 + 1) = (number-1) * x_part_AE0 + x_A;
    q(60 + (number-1)*6 + 2) = (number-1) * y_part_AE0 + y_A;
    q(60 + (number-1)*6 + 6) = q(54);
end
    
x_L = q(43);y_L = q(44);%L���ʼ����
x_J = x_L-10000;y_J = y_L;%J���ʼ����
x_part_LJ = (x_J - x_L)/(number_LJ + 1); %LJ���سɵ�С����x�����ϵľ��Գ���
y_part_LJ = (y_J - y_L)/(number_LJ + 1); %LJ���سɵ�С����y�����ϵľ��Գ���
%���˵�һ���⣬LJ���سɵ���������С�εĳ�ʼ����ϵλ������
for number = 1:number_LJ
    q(60 + number_AE0*6 + (number-1)*6 + 1) = number * x_part_LJ + x_L;
    q(60 + number_AE0*6 + (number-1)*6 + 2) = number * y_part_LJ + y_L;
    q(60 + number_AE0*6 + (number-1)*6 + 6) = q(60);
end

dq = zeros(6*n,1);
% dq = [	0;0;0;%dr2 ���������ʱ��һ�׵��� %�ٶȳ�ʼֵ
% 		0;0;0;%omega2
% 		0;0;0;%dr3
% 		0;0;0;%omega3
% 		0;0;0;%dr4
% 		0;0;0;%omega4
% 		0;0;0;%dr5
% 		0;0;0;%omega5
% 		0;0;0;%dr6
% 		0;0;0;%omega6
% 		0;0;0;%dr7
% 		0;0;0;%omega7
% 		0;0;0;%dr8
% 		0;0;0;%omega8
% 		0;0;0;%dr9
% 		0;0;0;%omega9
%         0;0;0;%dr10
%         0;0;0 %omega10
% 		];
%F1 = 0;F2 = 87920*12;F3 = 50240*23;
F1 = 0;F2 = 1052663;F3 = 1682180; %����Ϊ��������������λN
%p1������תҺѹ���ѹ�� p2�²�Һѹϵͳѹ����p3�ϲ�Һѹϵͳѹ��
Q1 = 0;Q2 = 0;Q3 = 0;dQ1 = 0;dQ2 = 0;dQ3 = 0; %������ʱû��ʹ��
u = [F1;F2;F3]; 
F = zeros(n*6,1);

x0 = [q;dq]; %ode״̬��ʼֵ 12nx1

tic;
[t_set,x]=ode23tb(@(t,x)Multi_Body_Dynamic_con(t,x,F,u,r,L,number_AE0,number_LJ,m),[0 12],x0,opt);%ode���������

end

