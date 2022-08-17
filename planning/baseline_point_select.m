clc;clear; close all;
figure(1);
circle=[5,5,1.5]; pose=[5,5,pi/6];
hold on;
quiver(pose(1),pose(2),0.8*cos(pose(3)),0.8*sin(pose(3)),'g','LineWidth',2,'MaxHeadSize',1,'AutoScaleFactor',0.89,'AutoScale','off')
plot_circle([5,5],1.5); axis([0,10,0,10]);
plot(pose(1),pose(2),'g.','MarkerSize',18);
[x1,y1,x2,y2]=P1P2(pose,circle);
[left,right]= vert_cal(pose,0.7);
plot([x1,x2],[y1,y2],'r')
plot(x1,y1,'k.','MarkerSize',15); plot(x2,y2,'k.','MarkerSize',15);
plot(left(1),left(2),'m.','MarkerSize',15);
plot(right(1),right(2),'m.','MarkerSize',15)
text(x1-0.3,y1-0.3,'P_1')
text(x2-0.3,y2-0.3,'P_2')
text(pose(1)-0.3,pose(2)-0.3,'P^c_i')
text(left(1)-0.3,left(2)-0.3,'P_3')
text(right(1)-0.3,right(2)-0.3,'P_4')
plot_circle2([x1,y1],0.5);
plot_circle2([x2,y2],0.5);
saveas(gca,'baseline_case1.eps','psc2');

figure(2);
circle=[5,5,1.5]; pose=[4,4,pi/4];
hold on;
quiver(pose(1),pose(2),0.8*cos(pose(3)),0.8*sin(pose(3)),'g','LineWidth',2,'MaxHeadSize',1,'AutoScaleFactor',0.89,'AutoScale','off')
plot_circle([5,5],1.5); axis([0,10,0,10]);
plot(4,4,'g.','MarkerSize',18);
[x1,y1,x2,y2]=P1P2(pose,circle);
[left,right]= vert_cal(pose,1.7);
plot([left(1),right(1)],[left(2),right(2)],'r')
plot(x1,y1,'k.','MarkerSize',15); plot(x2,y2,'k.','MarkerSize',15);
plot(left(1),left(2),'m.','MarkerSize',15);
plot(right(1),right(2),'m.','MarkerSize',15)
text(x1-0.3,y1-0.3,'P_1')
text(x2-0.3,y2-0.3,'P_2')
text(pose(1)-0.3,pose(2)-0.3,'P^c_i')
text(left(1)-0.3,left(2)-0.3,'P_3')
text(right(1)-0.3,right(2)-0.3,'P_4')
plot_circle2([left(1),left(2)],0.5);
plot_circle2([right(1),right(2)],0.5);
saveas(gca,'baseline_case2.eps','psc2');


figure(3);
circle=[5,5,1.5]; 
pose=[3.5,4,2*pi/5];
hold on;
quiver(pose(1),pose(2),0.8*cos(pose(3)),0.8*sin(pose(3)),'g','LineWidth',2,'MaxHeadSize',1,'AutoScaleFactor',0.89,'AutoScale','off')
plot_circle([5,5],1.5); axis([0,10,0,10]);
plot(pose(1),pose(2),'g.','MarkerSize',18);
[x1,y1,x2,y2]=P1P2(pose,circle);
[left,right]= vert_cal(pose,1.1);
plot([left(1),x2],[left(2),y2],'r')
plot(x1,y1,'k.','MarkerSize',15); plot(x2,y2,'k.','MarkerSize',15);
plot(left(1),left(2),'m.','MarkerSize',15);
plot(right(1),right(2),'m.','MarkerSize',15)
text(x1-0.3,y1-0.3,'P_1')
text(x2-0.3,y2-0.3,'P_2')
text(left(1)-0.3,left(2)-0.3,'P_3')
text(right(1)-0.3,right(2)-0.3,'P_4')
text(pose(1)-0.3,pose(2)-0.3,'P^c_i')
plot_circle2([left(1),left(2)],0.5);
plot_circle2([x2,y2],0.5);
saveas(gca,'baseline_case3.eps','psc2');




function [x1,y1,x2,y2]=P1P2(pose,circle)
x=pose(1); y=pose(2);
k=tan(pose(3)+pi/2);
b=pose(2)-k*pose(1);
aa=1+k^2;
bb=-2*circle(1)+2*k*(b-circle(2));
cc= circle(1)^2+(b-circle(2))^2-circle(3)^2;   
x1=1/(2*aa)*(-bb-sqrt(bb^2-4*aa*cc));
y1=y+(x1-x)*k;
x2=1/(2*aa)*(-bb+sqrt(bb^2-4*aa*cc));
y2=y-(x-x2)*k; 
end
function plot_circle(center, radius)
theta = linspace(0,2*pi,100);
x=center(1); y=center(2);
xc = center(1) + radius*cos(theta);
yc = center(2) + radius*sin(theta);
plot(x,y,'r*',xc,yc,'b-')
end
function plot_circle2(center, radius)
theta = linspace(0,2*pi,100);
x=center(1); y=center(2);
xc = center(1) + radius*cos(theta);
yc = center(2) + radius*sin(theta);
plot(xc,yc,'c-')
end
function [left,right]=vert_cal(pose,l)
   att=pose(3)+pi/2;
   left=[pose(1)+l*cos(att),pose(2)+l*sin(att)];
   right=[pose(1)-l*cos(att),pose(2)-l*sin(att)];
end