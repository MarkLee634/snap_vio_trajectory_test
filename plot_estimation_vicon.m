raw_data = csvread('compare_odom.csv', 30, 0);
time = raw_data(:,1);
estX = raw_data(:,2);
estY = raw_data(:,3);
estZ = raw_data(:,4);

trueX = raw_data(:,5);
trueY = raw_data(:,6);
trueZ = raw_data(:,7);

figure(1)
grid on 
hold on
plot3(trueX, trueY, trueZ, '-r', 'DisplayName', 'true')
plot3(estX, estY, estZ, '-g', 'DisplayName', 'snapvio UKF')
hold off

xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
legend show