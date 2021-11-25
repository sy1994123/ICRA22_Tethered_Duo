clc;clear; close all;
load('Traj_30.mat');
build_environemnt;
figure(1)
plot(Traj(:,1),Traj(:,2),'r',Traj(:,4),Traj(:,5),'m');
saveas(gca,'optimized_traj_30.eps','psc2');
load('Traj_10.mat');
build_environemnt;
figure(2);
plot(Traj(:,1),Traj(:,2),'r',Traj(:,4),Traj(:,5),'m');
saveas(gca,'optimized_traj_10.eps','psc2');
load('Traj_2085.mat');
build_environemnt;
figure(3)
plot(Traj(:,1),Traj(:,2),'r',Traj(:,4),Traj(:,5),'m');
saveas(gca,'optimized_traj_20.eps','psc2');
function []=build_environemnt
figure;
build_cost_map;
objects = [15, 67, 1;            
           21, 82, 2;
           32, 73, 3;
           35, 85, 4;
           18, 31, 4;
           19, 33, 5;
           80, 54, 6;
           79, 56, 7;
           90, 85, 8;
           93, 88, 9;
           89, 86, 10;
           110, 20, 11;
           113, 22, 12];
for i=1:size(objects,1)
    plot(objects(i,1),objects(i,2),'rs'); hold on;
end
startPose = [10 5 pi/2]; 
endPose= [140 5 -pi/2];
plot(7,5,'m.','Markersize',12);plot(13,5,'m.','Markersize',12);
plot(137,5,'m.','Markersize',12);plot(143,5,'m.','Markersize',12)
waypoints=[]; circles=[];
[c,r]=minboundcircle(objects(5:6,1),objects(5:6,2));
waypoints=[waypoints;c];
circles=[circles;[c r+2]];
plot_circle(c,r+2);
[c,r]=minboundcircle(objects(1,1),objects(1,2));
waypoints=[waypoints;c];
circles=[circles;[c r+2]];
plot_circle(c,r+2);
[c,r]=minboundcircle(objects(2,1),objects(2,2));
waypoints=[waypoints;c];
circles=[circles;[c r+2]];
plot_circle(c,r+2);
[c,r]=minboundcircle(objects(3,1),objects(3,2));
waypoints=[waypoints;c];
circles=[circles;[c r+2]];
plot_circle(c,r+2);
[c,r]=minboundcircle(objects(4,1),objects(4,2));
waypoints=[waypoints;c];
circles=[circles;[c r+2]];
plot_circle(c,r+2);

[c,r]=minboundcircle(objects(9:11,1),objects(9:11,2));
waypoints=[waypoints;c];
circles=[circles;[c r+2]];
plot_circle(c,r+2);
[c,r]=minboundcircle(objects(7:8,1),objects(7:8,2));
waypoints=[waypoints;c];
circles=[circles;[c r+2]];
plot_circle(c,r+2);
[c,r]=minboundcircle(objects(12:13,1),objects(12:13,2));
waypoints=[waypoints;c];
circles=[circles;[c r+2]];
plot_circle(c,r+2);
end
function plot_circle(center, radius)
theta = linspace(0,2*pi,100);
x=center(1); y=center(2);
xc = center(1) + radius*cos(theta);
yc = center(2) + radius*sin(theta);
plot(x,y,'r*',xc,yc,'b-')
end