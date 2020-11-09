function plot_nodes(q,number_AE0,number_LJ)
% q = reshape(q,[numel(q),1]);
q = q';
n = numel(q)/6; %9
x = [];y = [];z = [];
plot_Sequence{1} = [1,5,4,2,3,6,1];
plot_Sequence{2} = [1,2];
plot_Sequence{3} = [1,2];
plot_Sequence{4} = [1,2];
plot_Sequence{5} = [1,2,4,3];
plot_Sequence{6} = [1,2];
plot_Sequence{7} = [1,2];
plot_Sequence{8} = [1,2];
plot_Sequence{9} = [1,2];
plot_Sequence{10} = [1,2,4,3];
plot_Sequence{11} = [1,2];

for j = 1:number_AE0
    plot_Sequence{11+j} = [1,2];
end

for j = 1:number_LJ
    plot_Sequence{11+number_AE0+j} = [1,2];
end

for i = 1:(n+1)
	if i == 1
		qe = zeros(6,1);
		dqe = zeros(6,1);
	else
		qe = q(6*(i-2)+1:6*(i-2)+6);
		dqe = zeros(6,1); %只需要求出各点的位置，因此不需要速度
		
	end
	
	Body(i).Joint = set_Joint(qe,dqe,i,number_AE0,number_LJ);
	for j = 1:numel(plot_Sequence{i})
		r = Body(i).Joint(plot_Sequence{i}(j)).r;
		x = [x,r(1)];y = [y,r(2)];z = [z,r(3)];
	end
	
	if i == 1
		hold off;
	else
		hold on;
	end
% 	plot3(x,y,z);
	plot(x,y);
	x = [];y = [];z = [];
end

axis([-5000,30000,-5000,30000,-5000,5000]);
% axis equal
xlabel('x');ylabel('y');zlabel('z');
grid on;

end