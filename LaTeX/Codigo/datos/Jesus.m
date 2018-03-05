x=datostotreal(:,1);
y=datostotreal(:,2);
z=datostotreal(:,3);
x3=datostotsim(:,1);
y3=datostotsim(:,2);
z3=datostotsim(:,3);
x1=table2array(x);
y1=table2array(y);
z1=table2array(z);
x2=table2array(x3);
y2=table2array(y3);
z2=table2array(z3);
figure 
hold on
plot3(x1,y1,z1,'b')
plot3(x2,y2,z2,'r')
grid on
axis([-10 15 -10 15 0 10]);
xlabel('x'), ylabel('y'), zlabel('z');

% figure 
% comet3(x1,y1,z1)

figure 
quiver3(x1,y1,z1,yaw4,pitch4,-roll4,'g')
scale=2;
% MarkerSize=0.5
hold on
quiver3(x2,y2,z2,yaw1,pitch1,-roll1,scale,'r')
axis([-10 15 -5 15 0 5]);
xlabel('x'), ylabel('y'), zlabel('z');
