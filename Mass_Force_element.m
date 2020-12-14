function [Mass,Force] = Mass_Force_element(r,L,qe,dqe,F,g,me)
% %求解质量阵与力阵
%rho = 8*10^-6;  %密度 kg/mm^3
%m_tot = pi*r^2*L*rho; %质量
m_tot = me;
B_r_0C = [L/2;0;0];   %body frame中质心位置
B_theta_0 = [	1/2*m_tot*r^2,			0,					0;
				0,						1/3*m_tot*L^2,		0;
				0,						0,					1/3*m_tot*L^2];%i系中关于reference point 0的转动惯量
%% R
phi = qe(4:6);
R = get_R(phi);%旋转矩阵
%% theta_0_0,r_0_0C
I_r_0C = R * B_r_0C; %转换到inertial frame
%%
omega = dqe(4:6);
%%
F_ext = F;
G = m_tot * [eye(3)*g;skew(I_r_0C)*g]; %重力
h = [-R*(skew(omega)*(skew(omega)*B_r_0C));-skew(omega)*B_theta_0*omega];
%%
Mass = [m_tot*eye(3),           -m_tot*R*skew(B_r_0C);
        -m_tot*skew(B_r_0C)'*R',    B_theta_0];
%Force = -(F_ext + G + h);
Force = F_ext + G + h;
end
