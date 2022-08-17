 load('Traj_turtle_25.mat');
 plot(Traj(:,1),Traj(:,2),'r.'); hold on;
 plot(Traj(:,4),Traj(:,5),'r.');
 traj2=[interp(Traj(:,1),50),interp(Traj(:,2),50),interp(Traj(:,4),50),interp(Traj(:,5),50)]/10;
 traj2=traj2(1:end-50,:);
 plot(traj2(:,1),traj2(:,2),'r.'); hold on;
 plot(traj2(:,3),traj2(:,4),'r.');
 dlmwrite('traj25.txt',traj2)