function plot_nodes_postprocessing(x,t,number_AE0,number_LJ) %x:166x108
n = 10 + number_AE0 + number_LJ;
close all;figure(1);hold off;
for i = 1:size(x,1)-1 %时间点
	q_plot = x(i,1:size(x,2)/2); %1x54
	fprintf('t=%d\n',t(i));
	plot_nodes(q_plot,number_AE0,number_LJ);
	
	deltat = t(i+1) - t(i);
	pause(deltat);
end
%AE0截断后的角度误差
figure(2);
subplot(2,1,1)
i=6;j=54;%2号杆与10号杆对应的角度在x中的位置
plot_error(x,t,i,j);
title("A0D")
%LJ截断后的角度误差
subplot(2,1,2)
i=24;%5号杆对应的角度在x中的位置
if number_LJ == 0
    j = 60;
else
    j = 6*n;%11号杆的最后一部分对应的角度在x中的位置
end
plot_error(x,t,i,j);
title("DJ")
end

function plot_error(x,t,i,j)
q_plot2 = x(:,i);
plot(t,q_plot2','--');
legend("soll");
axis square;
xlabel("t/s");ylabel("phi/rad");
hold on; grid on;
q_plot10 = x(:,j);
plot(t,q_plot10');
legend("soll","ist");
end