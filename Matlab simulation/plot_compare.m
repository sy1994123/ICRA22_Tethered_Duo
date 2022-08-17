close all;
x_ref=out.xr.data;
x1=out.x1.data;
x2=out.x2.data;
t=out.x2.Time;
plot(t,x_ref(:,1),'r--','LineWidth',2); hold on;
plot(t,x1(:,1),'r','LineWidth',2);
plot(t,x2(:,1),'r-.','LineWidth',2);
plot(t,x_ref(:,2),'b--','LineWidth',2);
plot(t,x1(:,2),'b','LineWidth',2);
plot(t,x2(:,2),'b-.','LineWidth',2);
xlabel('Time/ s','FontSize',20,'Fontname', 'Times New Roman'); 
ylabel('Position/ m','FontSize',20,'Fontname', 'Times New Roman');
columnlegend(2,{'Xref','Xmrac','Xnorm','Yref','Ymrac','Ynorm'},'Location','NorthWest','FontSize',15,'boxon');
axis([0,15,0,4.5])
saveas(gca,'compare_mrac.eps','psc2');