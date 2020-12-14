function [Force] = add_Driving(u,Force,q,Body)
%添加驱动力 %%u代表输入变量--被控量--驱动力
%驱动力沿着x轴方向
Bi = 3;uj = 2;Jk = 2;direction = -1;
%Bi代表Body编号，uj代表u的参数编号，Jk代表joint编号，方向为x轴正负向
[F_drive] = set_Driving(Bi,uj,Jk,u,q,Body,direction);
Force = Force + F_drive;
%把力加到力阵中
Bi = 4;uj = 2;Jk = 2;direction = -1;
[F_drive] = set_Driving(Bi,uj,Jk,u,q,Body,direction);
Force = Force + F_drive;

Bi = 8;uj = 3;Jk = 2;direction = -1;
[F_drive] = set_Driving(Bi,uj,Jk,u,q,Body,direction);
Force = Force + F_drive;

Bi = 9;uj = 3;Jk = 2;direction = -1;
[F_drive] = set_Driving(Bi,uj,Jk,u,q,Body,direction);
Force = Force + F_drive;
end

function [Fk] = set_Driving(i,j,k,u,q,Body,direction)
Fk = [direction*u(j);0;0]; %此行重要---最优控制中定义控制变量u
 %局部坐标系下力的大小和方向--沿着x轴方向，direction判断方向

qe = q(6*(i-2)+1:6*(i-2)+6);%确定i号body局部坐标系
phi = qe(4:6);  %角度
R = get_R(phi);%旋转矩阵

r_0k = Body(i).Joint(k).r;%连接点位置
F_0k = R * Fk;            %将力转到inertial frame

Fke = [F_0k;skew(r_0k)*F_0k];%见ppt9页集中力载菏
Fk = Body(i).T_qe_q' * Fke;%转换到全局坐标系
end