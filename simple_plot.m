simple_tether_centerline;
load('simle_traj_opt.mat')
plot(Traj(:,4),Traj(:,5),'r','LineWidth',2); hold on;
load('simle_traj_opt2.mat'); 
plot(Traj(:,1),Traj(:,2),'r','LineWidth',2);
text(2,3.7,'P^1_s')
text(6.5,3.7,'P^2_s')
text(4.2,3.7,'P^c_s')

text(26,27.5 ,'P^1_e')
text(26,22.5,'P^2_e')
text(26,25,'P^c_e')

text(2,13,{'Centerline','(C)'}); 
quiver(4,12,5*cos(-pi/4),5*sin(-pi/4),'g','LineWidth',2,'MaxHeadSize',1,'AutoScaleFactor',0.89,'AutoScale','off')
text(11,4.7,{'Baseline Trajectory (P) '});
quiver(12,6,4*cos(pi*10/11),4*sin(pi*10/11),'Color',[0 0.4470 0.7410],'LineWidth',2,'MaxHeadSize',1,'AutoScaleFactor',0.89,'AutoScale','off')
text(23,16,{'Optimal';'Trajectory ';'(P_{opt})'});
quiver(25,19,4*cos(1.8),4*sin(1.8),'r','LineWidth',2,'MaxHeadSize',1,'AutoScaleFactor',0.89,'AutoScale','off')

text(12,8,'O_1');
text(7,17.5,'O_2');
text(20,16,'O_3');
text(18,28,'O_4');
saveas(gca,'planning.eps','psc2');