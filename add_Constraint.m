function [g,B,dg,Tau] = add_Constraint(q,dq,Body,number_AE0,number_LJ)
%设置每一个节点上的约束，需要广义坐标，对于约束需要的参数有B,Tau,g,dg
%Revolute 平面铰链约束；Prismatic 平面滑动约束；Closed_Revolute 平面四杆机构冗余约束
[ge{1},Be{1},dge{1},Taue{1}] = set_constraint(q,dq,Body(1),2,Body(2),1,'Revolute');
[ge{2},Be{2},dge{2},Taue{2}] = set_constraint(q,dq,Body(2),2,Body(3),1,'Revolute');
[ge{3},Be{3},dge{3},Taue{3}] = set_constraint(q,dq,Body(3),2,Body(4),2,'Prismatic');
[ge{4},Be{4},dge{4},Taue{4}] = set_constraint(q,dq,Body(4),1,Body(1),3,'Closed_Revolute');

[ge{5},Be{5},dge{5},Taue{5}] = set_constraint(q,dq,Body(10),2,Body(5),1,'Revolute');
[ge{6},Be{6},dge{6},Taue{6}] = set_constraint(q,dq,Body(5),3,Body(6),1,'Revolute');
[ge{7},Be{7},dge{7},Taue{7}] = set_constraint(q,dq,Body(10),3,Body(7),1,'Revolute');
[ge{8},Be{8},dge{8},Taue{8}] = set_constraint(q,dq,Body(6),2,Body(7),2,'Closed_Revolute');

[ge{9},Be{9},dge{9},Taue{9}] = set_constraint(q,dq,Body(6),2,Body(8),1,'Revolute');
[ge{10},Be{10},dge{10},Taue{10}] = set_constraint(q,dq,Body(8),2,Body(9),2,'Prismatic');
[ge{11},Be{11},dge{11},Taue{11}] = set_constraint(q,dq,Body(5),2,Body(9),1,'Closed_Revolute');

[ge{12},Be{12},dge{12},Taue{12}] = set_constraint(q,dq,Body(5),2,Body(11),1,'Revolute');
%在AE0添加旋转关节而加入的限制
if number_AE0 == 0
    [ge{13},Be{13},dge{13},Taue{13}] = set_constraint(q,dq,Body(2),2,Body(10),1,'Revolute');
else
    [ge{13},Be{13},dge{13},Taue{13}] = set_constraint(q,dq,Body(2),2,Body(12),1,'Revolute');%与body2相连的部分
    [ge{14},Be{14},dge{14},Taue{14}] = set_constraint(q,dq,Body(11+number_AE0),2,Body(10),1,'Revolute');%与body10相连的部分
%迭代部分
    for j = 1:number_AE0-1
        [ge{14+j},Be{14+j},dge{14+j},Taue{14+j}] = set_constraint(q,dq,Body(11+j),2,Body(11+j+1),1,'Revolute');
    end
end
%在LJ添加旋转关节而加入的限制
if number_LJ >= 1
    [ge{13+number_AE0+1},Be{13+number_AE0+1},dge{13+number_AE0+1},Taue{13+number_AE0+1}] = set_constraint(q,dq,Body(11+number_AE0+1),1,Body(11),2,'Revolute');
    for j = 1:number_LJ-1
        [ge{13+number_AE0+1+j},Be{13+number_AE0+1+j},dge{13+number_AE0+1+j},Taue{13+number_AE0+1+j}] = set_constraint(q,dq,Body(11+number_AE0+j),2,Body(11+number_AE0+j+1),1,'Revolute');
    end
end

g = [];B = [];dg = [];Tau = []; 
%将所有约束整合起来
for i = 1:numel(ge)
	g = [g;ge{i}];
	B = [B;Be{i}];
	dg = [dg;dge{i}];
	Tau = [Tau;Taue{i}];
end

end

