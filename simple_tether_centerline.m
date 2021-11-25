%% Multi-Robot Sensor Example
% Copyright 2018-2019 The MathWorks, Inc.
clc;clear; close all;
figure(1);
simple_build_cost_map;
%% Define waypoints, objects, and initial poses
objects = [10, 10, 1;            
           11, 16, 2;
           18, 18, 3;
           20, 25, 4];
objects2 = [9.5, 9.5,;
            10.5,10.5
            11.7, 15.7;
            10.3, 16.3,
            18.5, 18.5;
            17.6, 17.5;
            21, 25.3;
            19, 24.7];
figure(1);
for i=1:size(objects2,1)
    plot(objects2(i,1),objects2(i,2),'rs'); hold on;
end
startPose = [5 5 pi/2]; 
endPose= [25 25 0];
plot(3.5,5,'m.','Markersize',20);plot(6.5,5,'m.','Markersize',20);
plot(25,26.5,'m.','Markersize',20);plot(25,23.5,'m.','Markersize',20)
saveas(gca,'planning_1.eps','psc2');
waypoints=[]; circles=[];
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

figure(1);
plot(5,5,'bo');plot(25,25,'bo');
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
    planner = plannerHybridAStar(validator,'MinTurningRadius',1.3,'InterpolationDistance',0.1);
    refpath = plan(planner,startPose,goalPose);
    i=i+1;
    startPose=goalPose;
    %figure(2);
    %show(planner);
    path=[path;refpath.States];
end

figure(1); hold on;
plot(path(:,1),path(:,2),'g--','LineWidth',1.5);

%%%%%%%%%%%%%%%%%%%%%%%%%%% two robots planning
i=1; car1=[];car2=[]; j=1;
ll=1.5*ones(size(path,1),1);
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
%              plot(path2(1,1),path2(1,2),'r.');
%              plot(path2(1,4),path2(1,5),'m.') 
             k=k+1;
       end
       e_points(m)=size(car1,1);
       i=k;
       j=j+1;      
   else
       [l,r]=vert_cal(path(i,:),ll(i,1));
       car1=[car1;[l path(i,3)]];
       car2=[car2;[r path(i,3)]];
%        plot(l(1),l(2),'r.');
%        plot(r(1),r(2),'m.')  
       i=i+1;
   end 
end
Traj=[car1 car2];
plot(Traj(:,1),Traj(:,2),'.','Color',[0 0.4470 0.7410]);
plot(Traj(:,4),Traj(:,5),'.','Color',[0 0.4470 0.7410]);
function [path]=subplan(pose,circle,l,~)
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
     kk=dot([x1-x,y1-y],[cos(pose(3)+pi/4),sin(pose(3)+pi/4)]);   
     if (kk<0) 
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
       x4=pose(1)-l*cos(pose(3)+pi/2);
       y4=pose(2)-l*sin(pose(3)+pi/2);
       if ((l1>l) || (points_inside_circle2([x3,y3,pose(3)],circle))) && ((x1-pose(1))*(l*cos(pose(3)+pi/2))>=0) &&((y1-pose(2))*(l*sin(pose(3)+pi/2))>=0)
            path(1,1:3)=[x1,y1,pose(3)];
       else
            path(1,1:3)=[x3,y3,pose(3)];
       end

        if ((l2>l) || (points_inside_circle2([x4,y4,pose(3)],circle)))  && ((x2-pose(1))*(-l*cos(pose(3)+pi/2))>=0) &&((y2-pose(2))*(-l*sin(pose(3)+pi/2))>=0)
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
