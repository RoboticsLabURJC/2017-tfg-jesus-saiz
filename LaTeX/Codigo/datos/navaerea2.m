clear all

Cl1='Cl1.xlsx';
w_ew= xlsread(Cl1,2, 'E4:G504');

dif_t=0.01;

eul=zeros(500,3);
PQR =zeros(500,3);
q=zeros(500,4);

eul(1,:)=[15,60,30]*pi/180;
q(1,:)=angle2quat(eul(1,1),eul(1,2),eul(1,3));

for i=1:500
   PQR(i, :) = w_ew(i,:)*dif_t;
   P=PQR(i,1);
   Q=PQR(i,2);
   R=PQR(i,3);
   s=[0   -P/2 -Q/2 -R/2;
      P/2   0   R/2 -Q/2;
      Q/2 -R/2   0   P/2;
      R/2  Q/2 -P/2   0];
   absv= norm(PQR,2);
   
   if(absv ~= 0)
     q(i+1,:)=(cos(absv/2)*eye(4) + (2/absv)*sin(absv/2)*s)*q(i,:)';
   else
     q(i+1,:)=cos(absv/2)*q(i,:);
   end
    [eul(i+1,1) eul(i+1,2) eul(i+1,3)]=quat2angle(q(i+1,:));
end

time= [0:0.01:5];

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

