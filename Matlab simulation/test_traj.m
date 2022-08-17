clc;clear;
t=0:0.1:2000;
wd=0.05; R=3; xg=2.5; yg=2.5;
xd=R*sin(2*wd*t);
yd=R*sin(wd*t);
plot(xd,yd); hold on;