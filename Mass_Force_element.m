function [Mass,Force] = Mass_Force_element(r,L,qe,dqe,F,g,me)
% %���������������
%rho = 8*10^-6;  %�ܶ� kg/mm^3
%m_tot = pi*r^2*L*rho; %����
m_tot = me;
r_1_0C = [L/2;0;0];   %iϵ������λ��
theta_1_0 = [	1/2*m_tot*r^2,			0,					0;
				0,						1/3*m_tot*L^2,		0;
				0,						0,					1/3*m_tot*L^2];%iϵ�й���reference point Q��ת������
%% R
phi = qe(4:6);
R = get_R(phi);%��ת����
%% theta_0_0,r_0_0C
r_0_0C = R * r_1_0C;
%%
omega = dqe(4:6);
%%
F_ext = F;
G = m_tot * [eye(3)*g;skew(r_0_0C)*g]; %����
h = [-R*(skew(omega)*(skew(omega)*r_1_0C));-skew(omega)*theta_1_0*omega];
%%
Mass = [m_tot*eye(3),           -m_tot*R*skew(r_1_0C);
        -m_tot*skew(r_1_0C)'*R',    theta_1_0];
Force = -(F_ext + G + h);
end
