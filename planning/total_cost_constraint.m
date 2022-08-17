% Function for evaluating the cost for each waypoint for a trajectory 
function [S,P] = total_cost_constraint(Traj)
% % % 

% for i=1:0.3:9.9
%     figure(3); hold on;
%     plot(i,barrier_cost(i),'r.');
% end
%%%%%%
lambda = 10;
[N,M,K]=size(Traj);
S=zeros(K,N);
%%dist cost
t1=5000;t2=300;
for i=1:K
    for j=2:N
        a=Traj(j,:,i)-Traj(j-1,:,i);
        S(i,j)=t1*(sqrt(a(1)^2+a(2)^2)+sqrt(a(4)^2+a(5)^2))+t2*(abs(a(3))+abs(a(6)));  %distance cost
        b=sqrt((Traj(j,1,i)-Traj(j,4,i))^2+(Traj(j,2,i)-Traj(j,5,i))^2);
        S(i,j)=S(i,j)+barrier_cost(b);
    end
end
%% weighted average of total cost
for i=1:K
    for j=1:N
        if (max(S(:,j)) - min(S(:,j))==0)
            temp = S(i,j);
        else
            temp= (S(i,j) - min(S(:,j)))/(max(S(:,j)) - min(S(:,j)));
        end
        P(i,j)=exp(-lambda*temp);
    end
end
    e=sum(P,1);
    for i=1:N
        if e(i)==0
            P(:,i)=zeros(size(P(:,i)));
        else
        P(:,i)=P(:,i)/e(i);
        end
    end
end
function [cost]=barrier_cost(d) 
    d_min=0; d_max=10;
    d_rest=6; 
    k_h1=20; 
    k_h2=15;
    if (d<d_min) || (d>=d_max)
        dd=min(abs(d_min-d),abs(d-d_max));
        cost= 1000+1000*(dd^2);
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
% function [cost]=intersect_test(a,b,c,d)
%   ax=a(1); ay=a(2);  bx=b(1); by=b(2);
%   ax=a(1); ay=a(2);  bx=b(1); by=b(2);
%   if(min(ax,bx)<=max(cx,d.x) && min(c.y,d.y)<=max(a.y,b.y)&&min(c.x,d.x)<=max(a.x,b.x) && min(a.y,b.y)<=max(c.y,d.y)) 
% 　　return true;
% 
% end