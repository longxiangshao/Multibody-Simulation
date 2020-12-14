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
title("body A0D")
%LJ截断后的角度误差
subplot(2,1,2)
i=24;%5号杆对应的角度在x中的位置
if number_LJ == 0
    j = 60;
else
    j = 6*n;%11号杆的最后一部分对应的角度在x中的位置
end
plot_error(x,t,i,j);
title("body DJ")

plot_velocity_AB0(x,t,n)
end

function plot_error(x,t,i,j)
q_plot2 = x(:,i);
plot(t,q_plot2','--');
legend("soll");
axis square;
xlabel("t/s");ylabel("theta/rad");
hold on; grid on;
q_plot10 = x(:,j);
plot(t,q_plot10');
legend("soll","ist");
end

function plot_velocity_AB0(x,t,n)
figure(3);
subplot(2,2,1);
x_A = x(:,7);y_A = x(:,8);x_B0 = x(:,13);y_B0 = x(:,14);
x_AB0 = x_A - x_B0;y_AB0 = y_A - y_B0;
s_AB0 = sqrt(x_AB0.*x_AB0 + y_AB0.*y_AB0) - 3000;
plot(t,s_AB0);
title("displacement AB0");
xlabel("t/s");ylabel("s/mm");

subplot(2,2,2);
v_A_x = x(:,6*n+7);v_A_y = x(:,6*n+8);
%v_B0_x=x(:,6*n+13);v_B0_y=x(:,6*n+14);
v_A = sqrt(v_A_x.*v_A_x+v_A_y.*v_A_y);
%v_B0=sqrt(v_B0_x.*v_B0_x + v_B0_y.*v_B0_y);
v_AB0 = v_A;
plot(t,v_AB0)
title("velocity AB0");
xlabel("t/s");ylabel("v/(mm/s)");

subplot(2,2,3);
x_L = x(:,43);y_L = x(:,44);x_H = x(:,37);y_H = x(:,38);
x_LH = x_L - x_H;y_LH = y_L - y_H;
s_LH = sqrt(x_LH.*x_LH + y_LH.*y_LH) - 2570.992;
plot(t,s_LH);
title("displacement LH");
xlabel("t/s");ylabel("s/mm");

subplot(2,2,4);
v_L_x = x(:,6*n+43);v_L_y = x(:,6*n+44);
v_H_x = x(:,6*n+37);v_H_y = x(:,6*n+38);
% v_L = sqrt(v_L_x.*v_L_x + v_L_y.*v_L_y);
% v_H = sqrt(v_H_x.*v_H_x + v_H_y.*v_H_y);
% v_LH = v_L - v_H;
v_LH = sqrt((v_L_x-v_H_x).^2+(v_L_y-v_H_y).^2);
plot(t,v_LH);
title("velocity LH");
xlabel("t/s");ylabel("v/(mm/s)");

end