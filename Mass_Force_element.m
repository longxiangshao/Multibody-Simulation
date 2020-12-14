function [Mass,Force] = Mass_Force_element(r,L,qe,dqe,F,g,me)
% %���������������
%rho = 8*10^-6;  %�ܶ� kg/mm^3
%m_tot = pi*r^2*L*rho; %����
m_tot = me;
B_r_0C = [L/2;0;0];   %body frame������λ��
B_theta_0 = [	1/2*m_tot*r^2,			0,					0;
				0,						1/3*m_tot*L^2,		0;
				0,						0,					1/3*m_tot*L^2];%iϵ�й���reference point 0��ת������
%% R
phi = qe(4:6);
R = get_R(phi);%��ת����
%% theta_0_0,r_0_0C
I_r_0C = R * B_r_0C; %ת����inertial frame
%%
omega = dqe(4:6);
%%
F_ext = F;
G = m_tot * [eye(3)*g;skew(I_r_0C)*g]; %����
h = [-R*(skew(omega)*(skew(omega)*B_r_0C));-skew(omega)*B_theta_0*omega];
%%
Mass = [m_tot*eye(3),           -m_tot*R*skew(B_r_0C);
        -m_tot*skew(B_r_0C)'*R',    B_theta_0];
%Force = -(F_ext + G + h);
Force = F_ext + G + h;
end
