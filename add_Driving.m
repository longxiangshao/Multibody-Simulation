function [Force,B_drive] = add_Driving(u,Force,q,r,Body,number)
B_drive = zeros(6*number,3); %与B的意义完全不同
%添加驱动力 %%u代表输入变量--被控量--此为压力---压力可转化为驱动力
%驱动力沿着x轴方向
Bi = 3;uj = 2;Jk = 2;direction = -1;
%Bi代表Body，uj代表控制变量第几个参数（F2），joint编号，方向为x轴负向液压驱动力
[F_drive,B_drive_k] = set_Driving(Bi,uj,Jk,u,q,r,Body,direction);
Force = Force - F_drive; %48x1
B_drive = B_drive + B_drive_k;
%把力加到力阵中
Bi = 4;uj = 2;Jk = 2;direction = -1;
[F_drive,B_drive_k] = set_Driving(Bi,uj,Jk,u,q,r,Body,direction);
Force = Force - F_drive;
B_drive = B_drive + B_drive_k;

Bi = 8;uj = 3;Jk = 2;direction = -1;
[F_drive,B_drive_k] = set_Driving(Bi,uj,Jk,u,q,r,Body,direction);
Force = Force - F_drive;
B_drive = B_drive + B_drive_k;

Bi = 9;uj = 3;Jk = 2;direction = -1;
[F_drive,B_drive_k] = set_Driving(Bi,uj,Jk,u,q,r,Body,direction);
Force = Force - F_drive;
B_drive = B_drive + B_drive_k;
end

function [Fk,Bk] = set_Driving(i,j,k,u,q,r,Body,direction)

%A = pi * (r(i-1) ^ 2)*10^-6; 
Fk = [direction*u(j);0;0]; %此行重要---最优控制中定义控制变量u
 %局部坐标系下力的大小和方向--沿着x轴方向，direction判断方向
T_ui_u = zeros(1,numel(u)); %1x3
T_ui_u(j) = 1; %选取u1,u2,u3中的哪一个
Bk = [direction;0;0]*T_ui_u; %将力放到body frame的x负方向，即[-F2;0;0]或[-F3;0;0]

qe = q(6*(i-2)+1:6*(i-2)+6);%确定i号body局部坐标系
phi = qe(4:6);  %角度
R = get_R(phi);%旋转矩阵

r0k = Body(i).Joint(k).r;%连接点位置
F0k = R * Fk;            %将力转到全局坐标系
B0k = R * Bk;            %B0k*u=零系中的驱动力

Fke = [F0k;skew(r0k)*F0k];%见ppt9页集中力载菏
Bke = [B0k;skew(r0k)*B0k]; %Bke*u=零系中的驱动力及其产生的扭矩
Fk = Body(i).T_qe_q' * Fke;%转力到全局坐标系，Fk48x1--叠加到力阵中,Fke 6x1
Bk = Body(i).T_qe_q' * Bke;%转换到48x6的entire dof中

% max(abs(Bk * u - Fk))
end