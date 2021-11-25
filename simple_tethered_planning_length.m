figure;close all;clear;
load('simle_traj.mat')
%origin obstacle map
A=zeros(30,30);
A(:,1)=ones(30,1);
A(:,30)=ones(30,1);
A(1,:)=ones(1,30);
A(30,:)=ones(1,30);
A(18:22,16:20)=ones(5,5);
A(8:11,8:11)=ones(4,4);
simple_build_cost_map;
plot(3.5,5,'m.','Markersize',12);plot(6.5,5,'m.','Markersize',12);
plot(25,26.5,'m.','Markersize',12);plot(25,23.5,'m.','Markersize',12)
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
for k=1:size(circles,1)
    plot_circle(circles(k,1:2),circles(k,3));
end
step=2;
N=size(Traj,1);
tm_init=1;
T = zeros(N,N);
for i=1:N
    T(i,i) = 1/tm_init^2;
end
A2 = zeros(N,N); % Finite differencing matrix 
%should be symmetric for the covariance to be symmetric
for i=2:N-1
    A2(i, i-1) = 1;
    A2(i,i) = -2;
    A2(i, i+1) = 1;
end
A2(1,1) = -2; A2(1,2) = 1;
A2(N,N-1) = -2;  A2(N,N) = 1;

D = T*A2;
R_in = D'*D; % PSD matrix that defines control costs 

A3=zeros(N-3,N-2);
for i=1:N-3
    A3(i, i) = -1;
    A3(i,i+1) = 1;
end
B3=ones(N-3,1);
A4=[A3;-A3]; B4=[B3;B3];
Traj(2,3)=10; Traj(2,6)=10;
tot=30;
while step<tot
    if (mod(step,2)==1)
        X_ini=Traj(2:end-1,1:3); X2_ini=Traj(2:end-1,4:6); 
        s=Traj(1,1:3); e=Traj(end,1:3);
    else 
        X_ini=Traj(2:end-1,4:6); X2_ini=Traj(2:end-1,1:3);
        s=Traj(1,4:6); e=Traj(end,4:6);
    end
    N=size(Traj,1)-2;
    options=optimoptions('fmincon','MaxFunctionEvaluations',1e6,'Display','iter','Algorithm','interior-point');
    X_opt=fmincon(@(X_opt)cost(X_opt,s,e,A,circles,X2_ini,R_in),X_ini,[],[],[],[],[],[],@(X_opt)mynonlcon(X_opt,X2_ini,circles,A4,B4,s,e),options);
    if (mod(step,2)==1)
        Traj(2:end-1,1:3)=X_opt;
        Traj(2,6)=X_opt(1,3);
    else
        Traj(2:end-1,4:6)=X_opt;
        Traj(2,3)=X_opt(1,3);
    end
    cost(X_opt,s,e,A,circles,X2_ini,R_in)
    X_opt(1,3)
    plot(Traj(:,1),Traj(:,2),'r.',Traj(:,4),Traj(:,5),'g.');
    step=step+1;
end
plot(Traj(:,1),Traj(:,2),'r',Traj(:,4),Traj(:,5),'r')
function c=cost(Traj,s,e,A,circles,Traj2,R_in)
%  plot(Traj(:,1),Traj(:,2),'r.')
 j1=Traj;j2=Traj2;
 Traj=[s;Traj;e];
 N=size(Traj,1);
 t1=300;t2=0; 
 %dist_cost
 dir(1)=pi/2;
 for j=2:N
        a=Traj(j,:)-Traj(j-1,:); 
        dir(j)=atan2(a(2),a(1));
        S(j)=t1*(sqrt(a(1)^2+a(2)^2))+t2*abs(dir(j)-dir(j-1)); %sqrt(a(4)^2+a(5)^2))+abs(a(6)));  %distance cost
 end
 %obstacle cost
 S2=zeros(1,N);
 S6=zeros(1,N);
 t3=1e7;
 for j=2:N   
     x=Traj(j,1);    y=Traj(j,2); 
   
       x2=30-floor(y);   y2=floor(x)+1;
       if x2<=0
           x2=1;
       end
       if x2>=30
           x2=30;
       end
       if y2>=30
           y2=30;
       end
       if y2<=0
           y2=1;
       end
       if (A(x2,y2)==1)
            S2(j)=1e7;
       else
            S2(j)=0;
       end
       for k=1:size(circles,1)
            dist=sqrt((x-circles(k,1))^2+(y-circles(k,2))^2);
            if dist<circles(k,3)
                S2(j)=t3*(1/dist-1/circles(k,3))^2; 
            end
            S6(j)=S6(j)+dist;
       end  
 end
 %too far cost
 d_max=Traj(2,3);
 S3=zeros(1,N-2);
  for j=1:N-2
        d=sqrt((j1(j,1)-j2(j,1))^2+(j1(j,2)-j2(j,2))^2); %sqrt(a(4)^2+a(5)^2))+abs(a(6)));  %distance cost
        S3(j)=barrier_cost(d,d_max) ;
  end
 sum4=1;t4=10; t5=1;
 for i=1:2 
   aa=Traj(:,i);
   sum4=t4*aa'*R_in*aa;
 end
 sum5=t5*dir*R_in*dir';
 S_1=sum(S);S_4=sum(sum4);
 S_3=sum(S3);
 S_2=sum(S2);
 S_6=0*sum(S6);
 c=S_1+S_2+S_3; %+S_4; %+S_6;
end
function [cost]=barrier_cost(d,d_max) 
    d_min=0.1*d_max;
    d_rest=2/pi*d_max; 
    k_h1=20; 
    k_h2=15;
    if (d<d_min) || (d>=d_max) || (d_min>d_rest)
        dd=min(abs(d_min-d),abs(d-d_max));
        cost=1e7;
    else
       if (d>=d_min) && (d<d_rest)
           r1=pi/(2*(d_rest-d_min));
           r2=-r1*d_rest;
           cost=k_h1*tan(r1*d+r2)^2;
       else    
           cost=k_h2*(d-d_rest)^2-(d-d_rest)^2/(d-d_max);
       end
    end
end
function [c,ceq]=mynonlcon(Traj,Traj2,circles,A4,B4,s,e)
   N=size(Traj,1);
   c(1:N,1)=-Traj(:,1);
   c(N+1:2*N,1)=-Traj(:,2);
   c(2*N+1:3*N,1)=Traj(:,1)-150;
   c(3*N+1:4*N,1)=Traj(:,2)-100;
   c(4*N+1,1)=-Traj(1,3)+4.5;
   ceq=[];
   M=size(circles,1);
   temp=ones(M,1);
  for i=1:N-1
         P1=Traj(i,:); P2=Traj(i+1,:);
         P3=Traj2(i,:); P4=Traj2(i+1,:);           
         x=[P1(1) P2(1) P3(1) P4(1)];
         y=[P1(2) P2(2) P3(2) P4(2)];
      for j=1:M
          if (min(x)<=circles(j,1)) && (min(y)<=circles(j,2)) && (max(x)>=circles(j,1)) && (max(y)>=circles(j,2))
              temp(j)=0;
          end
      end
  end
  ceq=temp;
%    for i=1:N
%        for j=1:M
%            d1=sqrt((Traj(i,1)-circles(j,1))^2+(Traj(i,2)-circles(j,2))^2);
%            d2=sqrt((Traj2(i,1)-circles(j,1))^2+(Traj2(i,2)-circles(j,2))^2);
%            if d1<P(j,3)
%                P(j,3)=d1;
%                P(j,1)=Traj(i,1);
%                P(j,2)=Traj(i,2);
%            end
%           if d2<P(j,6)
%                P(j,6)=d2;
%                P(j,4)=Traj2(i,1);
%                P(j,5)=Traj2(i,2);
%            end
%        end
%    end
%    for j=1:M
%        if P(j,1)>P(j,4)
%            k=P(j,1);
%            P(j,1)=P(j,4);
%            P(j,4)=k;
%        end
%        if P(j,2)>P(j,5)
%            k=P(j,2);
%            P(j,2)=P(j,5);
%            P(j,5)=k;
%        end 
%    end
%    ceq(1:M,1)=P(:,1)-circles(:,1);
%    ceq(M+1:2*M,1)=circles(:,1)-P(:,4);
%    ceq(2*M+1:3*M,1)=P(:,2)-circles(:,2);
%    ceq(3*M+1:4*M,1)=circles(:,2)-P(:,5);
     %ceq=[];
 d=zeros(N+1,1); 
    for i=1:N-1
       d(i,1)=sqrt((Traj(i,1)-Traj(i+1,1))^2+(Traj(i,2)-Traj(i+1,2))^2);  
    end 
    d(N,1)=sqrt((Traj(1,1)-s(1,1))^2+(Traj(1,2)-s(1,2))^2);
    d(N+1,1)=sqrt((Traj(N,1)-e(1,1))^2+(Traj(N,2)-e(1,2))^2);
%   c1=A4*Traj(:,1)-B4;
%    c2=A4*Traj(:,2)-B4;
   c2=d-2*ones(N+1,1);
   c=[c;c2];
end
function plot_circle(center, radius)
theta = linspace(0,2*pi,100);
x=center(1); y=center(2);
xc = center(1) + radius*cos(theta);
yc = center(2) + radius*sin(theta);
plot(x,y,'r*',xc,yc,'b-')
end