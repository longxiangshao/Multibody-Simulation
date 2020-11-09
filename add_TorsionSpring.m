function [F_TorSpr_vec] = add_TorsionSpring(q,dq,Body,number_AE0,number_LJ)
n = 10 + number_AE0 + number_LJ;
F_TorSpr = zeros(6*n,2+number_AE0+number_LJ);
F_TorSpr_vec = zeros(6*n,1);

if number_AE0 == 0
    bodyi = 10; bodyj = 2;
    [F_TorSpr(:,1)] = set_Torsion_AE0(bodyi,bodyj,q,dq,Body); %body i和body j之间添加torsion spring
else
    [F_TorSpr(:,1)] = set_Torsion_AE0(2,12,q,dq,Body);
    [F_TorSpr(:,2)] = set_Torsion_AE0(10,11+number_AE0,q,dq,Body);
    for i = 1:number_AE0-1
        [F_TorSpr(:,2+i)] = set_Torsion_AE0(11+i,11+i+1,q,dq,Body);
    end
end

bodyi = 5; bodyj = 11;%5号杆与11号杆连接处加入torsion spring
[F_TorSpr(:,2+number_AE0)] = set_Torsion_LJ(bodyi,bodyj,q,dq,Body);

if number_LJ >= 1
    [F_TorSpr(:,2+number_AE0+1)] = set_Torsion_LJ(11,11+number_AE0+1,q,dq,Body);
    for i = 1:number_LJ-1
        [F_TorSpr(:,2+number_AE0+1+i)] = set_Torsion_LJ(11+number_AE0+i,11+number_AE0+i+1,q,dq,Body);
    end
end
for i = 1:2+number_AE0+number_LJ%将所有扭矩加起来
    F_TorSpr_vec = F_TorSpr_vec + F_TorSpr(:,i);
end
end

function [F_TorSpr] = set_Torsion_AE0(i,j,q,dq,Body)%设置下杆AE0连接处的刚度和阻尼
k = 5*10^11; d = 10^8; %k = 8*10^9; d = 10^6;
[F_TorSpr] = set_Torsion(i,j,q,dq,Body,k,d);
end

function [F_TorSpr] = set_Torsion_LJ(i,j,q,dq,Body)%设置上杆LJ连接处的刚度和阻尼
k = 3*10^11; d = 10^7; %k = 8*10^9; d = 10^6;
[F_TorSpr] = set_Torsion(i,j,q,dq,Body,k,d);
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


