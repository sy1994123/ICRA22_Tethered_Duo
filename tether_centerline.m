%% Multi-Robot Sensor Example
% Copyright 2018-2019 The MathWorks, Inc.
clc;clear; close all;
figure(1);
build_cost_map;
figure(2);
build_cost_map;
%% Define waypoints, objects, and initial poses
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
figure(1);
for i=1:size(objects,1)
    plot(objects(i,1),objects(i,2),'rs'); hold on;
end
startPose = [10 5 pi/2]; 
endPose= [140 5 -pi/2];
plot(7,5,'m.','Markersize',12);plot(13,5,'m.','Markersize',12);
plot(135,5,'m.','Markersize',12);plot(145,5,'m.','Markersize',12)

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
figure(1);
saveas(gca,'environment.eps','psc2');
% dir=[0.8;0.5;0;-0.5;-0.8];
% waypoints=[waypoints dir];

% figure(2)
% for i=1:size(objects,1)
%     plot(objects(i,1),objects(i,2),'rs'); hold on;
% end
% startPose = [10 5 pi/2]; 
% endPose= [140 5 -pi/2];
% plot(10,5,'bo');plot(140,5,'bo');

% waypoints=[]; circles=[];
% [c,r]=minboundcircle(objects(4:5,1),objects(4:5,2));
% waypoints=[waypoints;c];
% circles=[circles;[c r+2]];
% plot_circle(c,r+2);
% [c,r]=minboundcircle(objects(1:3,1),objects(1:3,2));
% waypoints=[waypoints;c];
% circles=[circles;[c r+2]];
% plot_circle(c,r+2);
% [c,r]=minboundcircle(objects(8:10,1),objects(8:10,2));
% waypoints=[waypoints;c];
% circles=[circles;[c r+2]];
% plot_circle(c,r+2);
% [c,r]=minboundcircle(objects(6:7,1),objects(6:7,2));
% waypoints=[waypoints;c];
% circles=[circles;[c r+2]];
% plot_circle(c,r+2);
% [c,r]=minboundcircle(objects(11:12,1),objects(11:12,2));
% waypoints=[waypoints;c];
% circles=[circles;[c r+2]];
% plot_circle(c,r+2);
% dir=[0.8;0.5;0;-0.5;-0.8];
% waypoints=[waypoints dir];
% figure(1);
% saveas(gca,'environment.eps','psc2');
plot(10,5,'bo');plot(140,5,'bo');
i=1; path=[];
while i<=size(waypoints,1)+1
    if i==size(waypoints,1)+1
        goalPose=endPose;
    else
        if i==size(waypoints,1)
            xy2=endPose;
        else   
            xy2=waypoints(i+1,:);

        end
        xy=waypoints(i,:);           
        dir=atan2(xy2(2)-xy(2),xy2(1)-xy(1));
        waypoints(i,3)=dir;
        goalPose=waypoints(i,:);
    end
    validator = validatorOccupancyMap;
    validator.Map = map;
    planner = plannerHybridAStar(validator,'MinTurningRadius',3);
    refpath = plan(planner,startPose,goalPose);
    i=i+1;
    startPose=goalPose;
    figure(2);
    show(planner);
    path=[path;refpath.States];
end
figure(1); hold on;
plot(path(:,1),path(:,2),'g.');
saveas(gca,'centerline.eps','psc2');
%%%%%%%%%%%%%%%%%%%%%%%%%%% two robots planning
i=1; car1=[];car2=[]; j=1;
ll=3*ones(size(path,1),1);
% ll(119:126,1)=1;
s_points=[]; e_points=[];m=0;
while i<=size(path,1)
   if (j<=size(circles,1)) && (points_inside_circle(path(i,1:3),circles(j,:),ll(i)))
       k=i;    
       m=m+1;
       s_points(m)=size(car1,1);
       while points_inside_circle(path(k,1:3),circles(j,:),ll(k))
             path2=subplan(path(k,:),circles(j,:),ll(k),[car1(end,:) car2(end,:)]);       
             car1=[car1;path2(:,1:3)];
             car2=[car2;path2(:,4:6)]; 
             plot(path2(1,1),path2(1,2),'r.');
             plot(path2(1,4),path2(1,5),'m.') 
             k=k+1;
       end
       e_points(m)=size(car1,1);
       i=k;
       j=j+1;      
   else
       [l,r]=vert_cal(path(i,:),ll(i,1));
       car1=[car1;[l path(i,3)]];
       car2=[car2;[r path(i,3)]];
       plot(l(1),l(2),'r.');
       plot(r(1),r(2),'m.')  
       i=i+1;
   end 
end
Traj=[car1 car2];
saveas(gca,'baseline.eps','psc2');
figure(2);
STOMP(Traj,s_points,e_points);
saveas(gca,'optimized_traj.eps','psc2');
function [path]=subplan(pose,circle,l,pose_old)
   x=pose(1); y=pose(2);
   if pose(3)==0
      x1=x;x2=x;
      y1=circle(2)+sqrt(circle(3)^2-(x-circle(1))^2);
      y2=circle(2)-sqrt(circle(3)^2-(x-circle(1))^2);
      l1=sqrt(circle(3)^2-(x-circle(1))^2); 
      l2=sqrt(circle(3)^2-(x-circle(1))^2);
   else
     k=tan(pose(3)+pi/2);
     b=pose(2)-k*pose(1);
%    i=0:1:30; j=k.*i+b;
%    plot(i,j,'k.');    
     aa=1+k^2;
     bb=-2*circle(1)+2*k*(b-circle(2));
     cc= circle(1)^2+(b-circle(2))^2-circle(3)^2;   
       x1=1/(2*aa)*(-bb-sqrt(bb^2-4*aa*cc));
       y1=y+(x1-x)*k;
       x2=1/(2*aa)*(-bb+sqrt(bb^2-4*aa*cc));
       y2=y-(x-x2)*k;   
     dist1=(x1-pose_old(1))^2+(y1-pose_old(2))^2;
     dist2=(x1-pose_old(4))^2+(y1-pose_old(5))^2;
     if dist2<dist1
       x1=1/(2*aa)*(-bb+sqrt(bb^2-4*aa*cc));
       y1=y+(x1-x)*k;
       x2=1/(2*aa)*(-bb-sqrt(bb^2-4*aa*cc));
       y2=y-(x-x2)*k;  
     end    
%        plot(x1,y1,'y*'); plot(x2,y2,'y*');
       l1=abs((x-x1)/cos(pose(3)+pi/2));        
       l2=abs((x2-x)/cos(pose(3)+pi/2)); 
   end
       x3=pose(1)+l*cos(pose(3)+pi/2);
       y3=pose(2)+l*sin(pose(3)+pi/2);
       if ((l1>l) || (points_inside_circle2([x3,y3,pose(3)],circle))) && ((x1-pose(1))*(l*cos(pose(3)+pi/2))>0) &&((y1-pose(2))*(l*sin(pose(3)+pi/2))>0)
            path(1,1:3)=[x1,y1,pose(3)];
       else
            path(1,1:3)=[x3,y3,pose(3)];
       end
        x4=pose(1)-l*cos(pose(3)+pi/2);
        y4=pose(2)-l*sin(pose(3)+pi/2);
        if ((l2>l) || (points_inside_circle2([x4,y4,pose(3)],circle))) && ((x2-pose(1))*(-l*cos(pose(3)+pi/2))>0) &&((y2-pose(2))*(-l*sin(pose(3)+pi/2))>0)
            path(1,4:6)=[x2,y2,pose(3)];
        else
            path(1,4:6)=[x4,y4,pose(3)];
        end
   
end
function [left,right]=vert_cal(pose,l)
   att=pose(3)+pi/2;
   left=[pose(1)+l*cos(att),pose(2)+l*sin(att)];
   right=[pose(1)-l*cos(att),pose(2)-l*sin(att)];
end
function [t]=points_inside_circle2(pos,circles)
    x=pos(1,1);y=pos(1,2);
    t=0;
    if (x-circles(1))^2+(y-circles(2))^2<=circles(3)^2
            t=1;
    end   
end
function [t]=points_inside_circle(pos,circles,d)
    x=pos(1,1);y=pos(1,2);
    t=0;
    if (x-circles(1))^2+(y-circles(2))^2<=circles(3)^2
            t=1;
    end
    [l,r]=vert_cal(pos,d);
    if (l(1)-circles(1))^2+(l(2)-circles(2))^2<=circles(3)^2
            t=1;
    end
    if (r(1)-circles(1))^2+(r(2)-circles(2))^2<=circles(3)^2
            t=1;
    end
    
end
function plot_circle(center, radius)
theta = linspace(0,2*pi,100);
x=center(1); y=center(2);
xc = center(1) + radius*cos(theta);
yc = center(2) + radius*sin(theta);
plot(x,y,'r*',xc,yc,'b-')
end
