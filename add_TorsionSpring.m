function [F_TorSpr_vec] = add_TorsionSpring(q,dq,Body,number_AE0,number_LJ)
n = 10 + number_AE0 + number_LJ; %总杆件数（除去基座）
F_TorSpr = zeros(6*n,2+number_AE0+number_LJ);
F_TorSpr_vec = zeros(6*n,1);

%计算刚度
L_AE0 = 9700;d_AE0 = 300;m_AE0 = 970;F_AE0 = 0;dr_AE0 = 10^6;
if number_AE0 >= 2
    [k_AE0,k_AE0_end] = set_Stiffness(L_AE0,d_AE0,number_AE0,m_AE0,F_AE0);
else
    k_AE0 = 1.5*10^11;
end
L_LJ = 10000;d_LJ = 300;m_LJ = 1000;F_LJ = 0;dr_LJ = 10^6;
if number_LJ >= 2
    [k_LJ,k_LJ_end] = set_Stiffness(L_LJ,d_LJ,number_LJ,m_LJ,F_LJ);
else
    k_LJ = 1.5*10^11;
end

%计算旋转弹簧带来的扭矩
if number_AE0 == 0
    bodyi = 10; bodyj = 2;
    [F_TorSpr(:,1)] = set_Torsion(bodyi,bodyj,q,dq,Body,k_AE0,dr_AE0); %body i和body j之间添加torsion spring
else
    [F_TorSpr(:,1)] = set_Torsion(2,12,q,dq,Body,k_AE0,dr_AE0);
    if number_AE0 >= 4 %判断是否使用k_AE0_end
        [F_TorSpr(:,2)] = set_Torsion(10,11+number_AE0,q,dq,Body,k_AE0_end,dr_AE0);
    else
        [F_TorSpr(:,2)] = set_Torsion(10,11+number_AE0,q,dq,Body,k_AE0,dr_AE0);
    end
    for i = 1:number_AE0-1
        [F_TorSpr(:,2+i)] = set_Torsion(11+i,11+i+1,q,dq,Body,k_AE0,dr_AE0);
    end
end

bodyi = 5; bodyj = 11;%5号杆与11号杆连接处加入torsion spring
[F_TorSpr(:,2+number_AE0)] = set_Torsion(bodyi,bodyj,q,dq,Body,k_LJ,dr_LJ);

if number_LJ >= 1
    [F_TorSpr(:,2+number_AE0+1)] = set_Torsion(11,11+number_AE0+1,q,dq,Body,k_LJ,dr_LJ);
    for i = 1:number_LJ-1
        [F_TorSpr(:,2+number_AE0+1+i)] = set_Torsion(11+number_AE0+i,11+number_AE0+i+1,q,dq,Body,k_LJ,dr_LJ);
    end
    if number_LJ >= 4 %判断是否使用k_LJ_end
        [F_TorSpr(:,2+number_AE0+number_LJ)] = set_Torsion(11+number_AE0+number_LJ-1,11+number_AE0+number_LJ,q,dq,Body,k_LJ_end,dr_LJ);
    end
end
for i = 1:2+number_AE0+number_LJ%将所有扭矩加起来
    F_TorSpr_vec = F_TorSpr_vec + F_TorSpr(:,i);
end
end

function [F_TorSpr] = set_Torsion(i,j,q,dq,Body,k,d)
qe_i = q(6*(i-2)+1:6*(i-2)+6);
dqe_i = dq(6*(i-2)+1:6*(i-2)+6);
qe_j = q(6*(j-2)+1:6*(j-2)+6);
dqe_j = dq(6*(j-2)+1:6*(j-2)+6);

phi_i = qe_i(6);
phi_j = qe_j(6);
torque_i = k * (phi_j - phi_i);
torque_j = -torque_i;

dphi_i = dqe_i(6);
dphi_j = dqe_j(6);
damping =  d * (dphi_i - dphi_j);
torque_i = torque_i - damping;
torque_j = torque_j - damping;

Force_i = [[0;0;0];torque_i * [0;0;1]];
Force_j = [[0;0;0];torque_j * [0;0;1]];

F_TorSpr = Body(i).T_qe_q' * Force_i + Body(j).T_qe_q' * Force_j;
end
function [k,k_end] = set_Stiffness(L,d,N,m,F) %此函数用于计算Torsion spring刚度
lambda = 2.11*10^5; %弹性模量,单位N/mm2
v = 0.3; %泊松比
E = lambda*(1+v)*(1-2*v)/v; %杨氏模量，单位N/mm2

I = pi*d^4/64; %mm^4
g = 9.80665;
q = m*g/L; %N/mm
l = L/(2*(N+1)); %mm

X = 2*q*l^2 + 2*F*l; %Nmm
Y = q*(l^2)/3*(N*(N+1)*(2*N+1)-6)+ F*l*(N*(N+1)-2); %Nmm
H = 1/6*(8*N^3*q*l^2 + 12*N^2*F*l); %Nmm
D = 4*q*l^3 + 4*F*l^2; %Nmm^2
Z = q*l^3*(N^2*(N+1)^2-4) + 2*F*l^2/3*(N*(N+1)*(2*N+1)-6); %Nmm^2
P = 1/6*(12*N^4*q*l^2 + 16*N^3*F*l); %Nmm

k = E*I*(X*Z-D*Y)/(P*X*l^2-D*H*l); %Nmm
k_end = E*I*(X*Z-D*Y)/(H*Z*l-P*Y*l^2); %Nmm

end
