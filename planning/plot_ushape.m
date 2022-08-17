clc;clear;close all;
step=0.1;
k=1;
for i=1.2:step:9.5
    c(k)=barrier_cost(i);
    k=k+1;   
end
x=1.2:step:9.5;
plot(x,c,'r'); hold on;
axis([0,10,0,500]);
xlabel('d/m','FontSize',20,'Fontname', 'Times New Roman'); 
ylabel('Expansion Cost Je','FontSize',20,'Fontname', 'Times New Roman');
y=0:10:500;
x=1.1*ones(length(y),1); 
plot(x,y,'b.','MarkerSize',10);
plot(1.1,0,'b.','MarkerSize',10);
x=9.6*ones(length(y),1); 
plot(x,y,'b.','MarkerSize',10);
plot(2/pi*9.6,0,'b.','MarkerSize',15);

text(1.3,50,'dmin','FontSize',15,'Fontname', 'Times New Roman');
x=[0.27,0.21];
y=[0.18,0.13];
annotation('textarrow',x,y,'Color',[0.8500 0.3250 0.0980],'LineWidth',1.5);
text(8.3,50,'dmax','FontSize',15,'Fontname', 'Times New Roman');
x=[0.81,0.87];
y=[0.18,0.13];
annotation('textarrow',x,y,'Color',[0.8500 0.3250 0.0980],'LineWidth',1.5);
text(5,50,'drest','FontSize',15,'Fontname', 'Times New Roman');
x=[0.57,0.60];
y=[0.18,0.14];
annotation('textarrow',x,y,'Color',[0.8500 0.3250 0.0980],'LineWidth',1.5);
saveas(gca,'cost_u.eps','psc2');
function [cost]=barrier_cost(d) 
    d_max=10; d_min=0.1*d_max;
    d_rest=2/pi*d_max; 
    k_h1=8; %20
    k_h2=50; %15
    if (d<d_min) || (d>=d_max)
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