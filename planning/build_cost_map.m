clc;
A=zeros(100,150);
A(:,1)=ones(100,1);
A(:,150)=ones(100,1);
A(1,:)=ones(1,150);
A(100,:)=ones(1,150);
A(51:60,31:40)=ones(10,10);
A(71:90,51:80)=ones(20,30);
A(21:50,64:68)=ones(30,5);
A(51:54,101:130)=ones(4,30);
map = binaryOccupancyMap(A);
show(map);hold on;