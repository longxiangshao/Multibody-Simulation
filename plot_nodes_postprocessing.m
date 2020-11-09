function plot_nodes_postprocessing(x,t,number_AE0,number_LJ) %x:166x108
n = 10 + number_AE0 + number_LJ;
close all;figure(1);hold off;
for i = 1:size(x,1)-1 %ʱ���
	q_plot = x(i,1:size(x,2)/2); %1x54
	fprintf('t=%d\n',t(i));
	plot_nodes(q_plot,number_AE0,number_LJ);
	
	deltat = t(i+1) - t(i);
	pause(deltat);
end
%AE0�ضϺ�ĽǶ����
figure(2);
subplot(2,1,1)
i=6;j=54;%2�Ÿ���10�Ÿ˶�Ӧ�ĽǶ���x�е�λ��
plot_error(x,t,i,j);
title("A0D")
%LJ�ضϺ�ĽǶ����
subplot(2,1,2)
i=24;%5�Ÿ˶�Ӧ�ĽǶ���x�е�λ��
if number_LJ == 0
    j = 60;
else
    j = 6*n;%11�Ÿ˵����һ���ֶ�Ӧ�ĽǶ���x�е�λ��
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