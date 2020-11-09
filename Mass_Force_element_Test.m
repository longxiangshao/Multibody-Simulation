clear;clc;
format long; %���������д�����ʾ��ʽ
opt=odeset('RelTol',0.1,'AbsTol',5,'MaxStep',5);
number_AE0 = 0;%�������Ǹ�����AE0���ּ��뼸���ؽ�
number_LJ = 10;%�������Ǹ�����FL���ּ��뼸���ؽ�
[t_set,x] = Test(opt,number_AE0,number_LJ);
plot_nodes_postprocessing(x,t_set,number_AE0,number_LJ);

function [t_set,x] = Test(opt,number_AE0,number_LJ)
n = 10 + number_AE0 + number_LJ; %�˼�����������������

r = zeros(n,1)+20;% r = [20,20,20,20,20,20,20,20,20];�˼�Բ����뾶����λmm
L_part_AE0 = 7500 /(1 + number_AE0);%�����Ǹ�����AE0���ֱ��ؽڽضϳɵ�С�˵ĳ���
L_part_LJ = 7500 /(1 + number_LJ);%�����Ǹ�����LF���ֱ��ؽڽضϳɵ�С�˵ĳ���

L = zeros(n,1);
L_0 = [13000,2000,2000,13000,2000,1500,2000,2000];%�˸���ԭ����
%�������泤�˵ĳ���
L(1:8)=L_0;
L(1)=5000;%A0A
L(9)=500 + L_part_AE0;
L(10+1:10+number_AE0) = L_part_AE0;
%�������泤�˵ĳ���
L(4) = 5500;
L(10) = L_part_LJ;
L(10+number_AE0+1:10+number_AE0+number_LJ) = L_part_LJ;

q = zeros(6*n,1);
q(1:60) = [-868;1500;0;%0_r_2��A0   %48x1-�������� %���ڵ��ʼλ��(����������)
		0;0;deg2rad(20);%phi2������ڹ�������x�����ת�Ƕ�
		3830;3210;0;%0_r_3��A
		0;0;deg2rad(42.4-180);%phi3
		868;500;0;%0_r_4��B0
		0;0;deg2rad(42.4);%phi4
		11349;5947;0;%0_r_5��D
		0;0;deg2rad(160);%phi5
		11221;7057;0;%0_r_6��F
		0;0;deg2rad(-131);%phi6
		11221;4836;0;%0_r_7��E
		0;0;deg2rad(151);%phi7
		9905;5552;0;%0_r_8��H
		0;0;deg2rad(148);%phi8
		6181;7828;0;%0_r_9��L
		0;0;deg2rad(148-180);%phi9
        3830;3210;0;%0_r_10��A
        0;0;deg2rad(20);%phi10
        6181;7828;0;%0_r_11��L
        0;0;deg2rad(160);%phi11
		];
x_A = 3830;y_A = 3210;%A���ʼ����
x_E0 = 10879;y_E0 = 5776;%E0���ʼ����
x_part_AE0 = (x_E0 - x_A)/(number_AE0 + 1);%AE0���سɵ�С����x�����ϵľ��Գ���
y_part_AE0 = (y_E0 - y_A)/(number_AE0 + 1);%AE0���سɵ�С����y�����ϵľ��Գ���

q(49) = number_AE0 * x_part_AE0 + x_A;    
q(50) = number_AE0 * y_part_AE0 + y_A;
q(54) = deg2rad(20);

for number = 1:number_AE0
    q(60 + (number-1)*6 + 1) = (number-1) * x_part_AE0 + x_A;
    q(60 + (number-1)*6 + 2) = (number-1) * y_part_AE0 + y_A;
    q(60 + (number-1)*6 + 6) = deg2rad(20);
end
    
x_L = 6181;y_L = 7828;%L���ʼ����
x_J = -866;y_J = 10393;%J���ʼ����
x_part_LJ = (x_J - x_L)/(number_LJ + 1);
y_part_LJ = (y_J - y_L)/(number_LJ + 1);

for number = 1:number_LJ
    q(60 + number_AE0*6 + (number-1)*6 + 1) = number * x_part_LJ + x_L;
    q(60 + number_AE0*6 + (number-1)*6 + 2) = number * y_part_LJ + y_L;
    q(60 + number_AE0*6 + (number-1)*6 + 6) = deg2rad(160);
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
F1 = 0;F2 = 87920;F3 = 50240; %����Ϊ��������������λN
%p1������תҺѹ���ѹ�� p2�²�Һѹϵͳѹ����p3�ϲ�Һѹϵͳѹ��
Q1 = 0;Q2 = 0;Q3 = 0;dQ1 = 0;dQ2 = 0;dQ3 = 0; %������ʱû��ʹ��
u = [F1;F2;F3]; 
F = zeros(n*6,1);
% F = [	0;0;0;%F2 ��������
% 		0;0;0;%M2
% 		0;0;0;%F3
% 		0;0;0;%M3
% 		0;0;0;%F4
% 		0;0;0;%M4
% 		0;0;0;%F5
% 		0;0;0;%M5
% 		0;0;0;%F6
% 		0;0;0;%M6
% 		0;0;0;%F7
% 		0;0;0;%M7
% 		0;0;0;%F8
% 		0;0;0;%M8
% 		0;0;0;%F9
% 		0;0;0;%M9
%         0;0;0;%F10
%         0;0;0 %M10
% 		];

x0 = [q;dq]; %ode״̬��ʼֵ 108x1

tic;
[t_set,x]=ode23tb(@(t,x)Multi_Body_Dynamic_con(t,x,F,u,r,L,number_AE0,number_LJ),[0 10],x0,opt);%ode���������

end

