function [Force] = add_Driving(u,Force,q,Body)
%��������� %%u�����������--������--������
%����������x�᷽��
Bi = 3;uj = 2;Jk = 2;direction = -1;
%Bi����Body��ţ�uj����u�Ĳ�����ţ�Jk����joint��ţ�����Ϊx��������
[F_drive] = set_Driving(Bi,uj,Jk,u,q,Body,direction);
Force = Force + F_drive;
%�����ӵ�������
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
Fk = [direction*u(j);0;0]; %������Ҫ---���ſ����ж�����Ʊ���u
 %�ֲ�����ϵ�����Ĵ�С�ͷ���--����x�᷽��direction�жϷ���

qe = q(6*(i-2)+1:6*(i-2)+6);%ȷ��i��body�ֲ�����ϵ
phi = qe(4:6);  %�Ƕ�
R = get_R(phi);%��ת����

r_0k = Body(i).Joint(k).r;%���ӵ�λ��
F_0k = R * Fk;            %����ת��inertial frame

Fke = [F_0k;skew(r_0k)*F_0k];%��ppt9ҳ�������غ�
Fk = Body(i).T_qe_q' * Fke;%ת����ȫ������ϵ
end