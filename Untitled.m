close all
figure(2);
q_plot2 = x(:,6);
plot(t_set,q_plot2','--');
legend("soll");
axis square;
xlabel("t/s");ylabel("phi/rad");
hold on; grid on;
q_plot10 = x(:,54);
plot(t_set,q_plot10');
legend("soll","ist");