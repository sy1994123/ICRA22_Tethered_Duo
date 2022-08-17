 load('Traj.mat');
 traj2=[interp(Traj(:,1),50),interp(Traj(:,2),50),interp(Traj(:,4),50),interp(Traj(:,5),50)]/10;
 dlmwrite('traj.txt',traj2)