clear all

dif_t=0.01;

eul=zeros(500,3);
q=zeros(500,4);
r=eul;
v=r;

Cl1='Cl1.xlsx';
f_ef= xlsread(Cl1,2, 'B4:D504');
w_ew= xlsread(Cl1,2, 'E4:G504');


eul(1,:)=[15,60,30]*pi/180;
q(1,:)=angle2quat(eul(1,1),eul(1,2),eul(1,3));
r(1,:)=[0 0 0];
v(1,:)=r(1,:);
gi=[0 0 9.81];

for i=1:500
    C_bi=quat2dcm(q(i,:));
    C_ib=C_bi';
    Q=[-q(i,2) -q(i,3) -q(i,4); 
        q(i,1)  q(i,4) -q(i,3);
       -q(i,4)  q(i,1)  q(i,2);
        q(i,3) -q(i,2)  q(i,1)];
    r(i+1,:)=dif_t*v(i,:)+ r(i,:);
    v(i+1,:)=((C_ib*f_ef(i,:)' + gi')*dif_t)' + v(i ,:);
    q(i+1,:)=((0.5*Q*C_ib*w_ew(i,:)')*dif_t)' + q(i,:);
    [eul(i+1,1) eul(i+1,2) eul(i+1,3)]=quat2angle(q(i+1,:));
    
end

time= [0:0.01:5];

plot3(r(:,1),r(:,2),-r(:,3),'r')
axis([0 abs(r(500,1)) 0 abs(r(500,2)) 0 abs(r(500,3))])
grid on
title ('trayectoria sistema cuerpo')
xlabel('x')
ylabel('y')
zlabel('z')

plot(eul(:,1),time,'DisplayName','yaw')
title ('angulos de euler restituidos')
xlabel('angulo(º)')
ylabel('Tiempo(seg)')
grid on

hold on
plot(eul(:,2),time,'DisplayName','pitch')
hold on
plot(eul(:,3),time,'DisplayName','roll')
legend('show')
hold off

plot3(abs(eul(:,1)),eul(:,2),eul(:,3),'g')
grid on
title ('variacion de angulos de euler 3D')
xlabel('yaw')
ylabel('pitch')
zlabel('roll')