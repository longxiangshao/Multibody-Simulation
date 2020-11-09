function [Force,B_drive] = add_Driving(u,Force,q,r,Body,number)
B_drive = zeros(6*number,3); %��B��������ȫ��ͬ
%��������� %%u�����������--������--��Ϊѹ��---ѹ����ת��Ϊ������
%����������x�᷽��
Bi = 3;uj = 2;Jk = 2;direction = -1;
%Bi����Body��uj������Ʊ����ڼ���������F2����joint��ţ�����Ϊx�Ḻ��Һѹ������
[F_drive,B_drive_k] = set_Driving(Bi,uj,Jk,u,q,r,Body,direction);
Force = Force - F_drive; %48x1
B_drive = B_drive + B_drive_k;
%�����ӵ�������
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
Fk = [direction*u(j);0;0]; %������Ҫ---���ſ����ж�����Ʊ���u
 %�ֲ�����ϵ�����Ĵ�С�ͷ���--����x�᷽��direction�жϷ���
T_ui_u = zeros(1,numel(u)); %1x3
T_ui_u(j) = 1; %ѡȡu1,u2,u3�е���һ��
Bk = [direction;0;0]*T_ui_u; %�����ŵ�body frame��x�����򣬼�[-F2;0;0]��[-F3;0;0]

qe = q(6*(i-2)+1:6*(i-2)+6);%ȷ��i��body�ֲ�����ϵ
phi = qe(4:6);  %�Ƕ�
R = get_R(phi);%��ת����

r0k = Body(i).Joint(k).r;%���ӵ�λ��
F0k = R * Fk;            %����ת��ȫ������ϵ
B0k = R * Bk;            %B0k*u=��ϵ�е�������

Fke = [F0k;skew(r0k)*F0k];%��ppt9ҳ�������غ�
Bke = [B0k;skew(r0k)*B0k]; %Bke*u=��ϵ�е����������������Ť��
Fk = Body(i).T_qe_q' * Fke;%ת����ȫ������ϵ��Fk48x1--���ӵ�������,Fke 6x1
Bk = Body(i).T_qe_q' * Bke;%ת����48x6��entire dof��

% max(abs(Bk * u - Fk))
end