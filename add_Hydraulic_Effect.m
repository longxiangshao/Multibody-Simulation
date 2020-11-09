function Force = add_Hydraulic_Effect(q,dq,Force,Body)
n = numel(q) / 6;
%% 液压影响，液压弹簧阻尼
k =0;%弹簧系数k--根据需要修改--注意单位
r0 = 3500;
b1 = 3; j1 = 2; b2 = 4; j2 = 2;
Force = Force - set_Hydraulic_spring(n,r0,b1,j1,b2,j2,Body,k);

k = 0;
r0 = 2300;
b1 = 8; j1 = 2; b2 = 9; j2 = 2;
Force = Force - set_Hydraulic_spring(n,r0,b1,j1,b2,j2,Body,k);
%%
c = 0;%阻尼系数c--根据需要修改--注意单位
b1 = 3; j1 = 2; b2 = 4; j2 = 2;
Force = Force - set_Hydraulic_damping(n,b1,j1,b2,j2,Body,c);

c = 0;
b1 = 8; j1 = 2; b2 = 9; j2 = 2;
Force = Force - set_Hydraulic_damping(n,b1,j1,b2,j2,Body,c);

end

function F = set_Hydraulic_spring(n,r0,b1,j1,b2,j2,Body,k)
r = norm(Body(b1).Joint(j1).r - Body(b2).Joint(j2).r);
F = zeros(6*n,1);

b = b1;j = j1;
Fke = [k*(r-r0);0;0];
F0ke = get_R(Body(b).Joint(j).phi) * Fke;
Fe = [F0ke;skew(Body(b).Joint(j).r)*F0ke];
F = F + Body(b).T_qe_q' * Fe;

b = b2;j = j2;
Fke = [k*(r-r0);0;0];
F0ke = get_R(Body(b).Joint(j).phi) * Fke;
Fe = [F0ke;skew(Body(b).Joint(j).r)*F0ke];
F = F + Body(b).T_qe_q' * Fe;

end

function F = set_Hydraulic_damping(n,b1,j1,b2,j2,Body,c)
F = zeros(6*n,1);

b = b1;j = j1;br = b2;jr = j2;
dr = Body(br).Joint(jr).dr - Body(b).Joint(j).dr;
R = get_R(Body(b).Joint(j).phi);
x = R(:,1);
Fce = [c*x'*dr;0;0];
F0ce = R * Fce;
Fe = [F0ce;skew(Body(b).Joint(j).r)*F0ce];
F = F + Body(b).T_qe_q' * Fe;

b = b2;j = j2;br = b1;jr = j1;
dr = Body(br).Joint(jr).dr - Body(b).Joint(j).dr;
R = get_R(Body(b).Joint(j).phi);
x = R(:,1);
Fce = [c*x'*dr;0;0];
F0ce = R * Fce;
Fe = [F0ce;skew(Body(b).Joint(j).r)*F0ce];
F = F + Body(b).T_qe_q' * Fe;

end
