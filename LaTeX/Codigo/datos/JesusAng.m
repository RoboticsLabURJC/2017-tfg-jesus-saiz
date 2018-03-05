pitch3=datostotreal(:,4);
roll3=datostotreal(:,5);
yaw3=datostotreal(:,6);
tiempo=datostotreal(:,8);
time4=table2array(tiempo);
pitch4=table2array(pitch3)*(180/pi);
roll4=table2array(roll3)*(180/pi);
yaw4=table2array(yaw3)*(180/pi);


figure
plot(pitch4,time4,'b','DisplayName','pitch')
title ('Ángulos del drone reales')
xlabel('Ángulo(º)')
ylabel('Tiempo(seg)')
grid on
hold on
plot(roll4,time4,'r','DisplayName','roll')
hold on
plot(yaw4,time4,'g','DisplayName','yaw')
axis([-20 30 0 300]);
legend('show')

pitch=datostotsim(:,4);
roll=datostotsim(:,5);
yaw=datostotsim(:,6);
t=datostotsim(:,8);
pitch1=table2array(pitch)*(180/pi)+90;
roll1=table2array(roll)*(180/pi);
yaw1=table2array(yaw)*(180/pi)+90;
time=table2array(t);

figure
plot(pitch1,time,'b','DisplayName','pitch')
title ('Ángulos del drone con autoloc')
xlabel('Ángulo(º)')
ylabel('Tiempo(seg)')
grid on
hold on
plot(roll1,time,'r','DisplayName','roll')
hold on
plot(yaw1,time,'g','DisplayName','yaw')
legend('show')
