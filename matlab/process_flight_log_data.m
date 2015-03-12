

if(~exist('flight_log_data', 'var'))
    parse_flightlog_06('rover_flight_logs\rev_6\pd_roll_spin_test_0.txt');
    global flight_log_data;
    flight_log_data.rover_t = flight_log_data.rover_t(1:max(size(flight_log_data.rover_ex_gyro)));
end 

mag = 0;
acc = 0;
gyro = 0;
est = 0;
pids = 1;

noise_check  = 0;


%% mag cal
if mag
% close all;
% %scatter3(flight_log_data.rover_int_(:, 1), flight_log_data.rover_int_(:, 2), flight_log_data.rover_int_(:, 3), '.')
 [b,a] = butter(20,0.2);
 mag_x = filter(b,a,flight_log_data.rover_int_mag(:, 1));
 mag_y = filter(b,a,flight_log_data.rover_int_mag(:, 2));
 mag_z = filter(b,a,flight_log_data.rover_int_mag(:, 3));
% 
figure; hold on;
plot(flight_log_data.rover_t, flight_log_data.rover_int_mag(:, 1), 'b');
%plot(flight_log_data.rover_t, mag_x, 'r');
title(sprintf('X Max: %f, X Min: %f', max(mag_x), min(mag_x)));
%disp(sprintf('X Max: %f, X Min: %f', max(mag_x), min(mag_x)));

figure; hold on;
plot(flight_log_data.rover_t, flight_log_data.rover_int_mag(:, 2), 'b');
%plot(flight_log_data.rover_t, mag_y, 'r');
title(sprintf('Y Max: %f, Y Min: %f', max(mag_y), min(mag_y)));
%disp(sprintf('Y Max: %f, Y Min: %f', max(mag_y), min(mag_y)));

figure; hold on;
plot(flight_log_data.rover_t, flight_log_data.rover_int_mag(:, 3), 'b');
%plot(flight_log_data.rover_t, mag_z, 'r');

title(sprintf('Z Max: %f, Z Min: %f', max(mag_z), min(mag_z)));
% % disp(sprintf('Z Max: %f, Z Min: %f', max(mag_z), min(mag_z)));
% 
% figure;
% plot(flight_log_data.rover_int_mag(:, 1), flight_log_data.rover_int_mag(:, 2), '.');
% title('X vs. Y')
% 
% figure;
% plot(flight_log_data.rover_int_mag(:, 2), flight_log_data.rover_int_mag(:, 3), '.');
% title('Y vs. Z')
% 
% figure;
% plot(flight_log_data.rover_int_mag(:, 1), flight_log_data.rover_int_mag(:, 3), '.');
% title('X vs. Z')
% % 
 figure
 scatter3(flight_log_data.rover_int_mag(:, 1), flight_log_data.rover_int_mag(:, 2), flight_log_data.rover_int_mag(:, 3), '.')

%find soft iron errors

% mag_max = [611 - 53,  563 - 17, 1341 - 806];
% mag_min = [-505 - 53, -529 - 17, 271 - 806];
% mag_avgs = mean(mag_max);
% 
% mag_scale_x = mag_avgs/mag_max(1)
% mag_scale_y = mag_avgs/mag_max(2)
% mag_scale_z = mag_avgs/mag_max(3)

% mag_x = flight_log_data.rover_int_mag(:, 1) - 53;
% mag_y = flight_log_data.rover_int_mag(:, 2) - 34;
% mag_z = flight_log_data.rover_int_mag(:, 3) - 806;




end

%%__

%% accel cal
if acc
% close all;
% %scatter3(flight_log_data.rover_int_acc(:, 1), flight_log_data.rover_int_acc(:, 2), flight_log_data.rover_int_acc(:, 3), '.')
[b,a] = butter(9,0.2);
acc_x = filter(b,a,flight_log_data.rover_int_acc(:, 1));
acc_y = filter(b,a,flight_log_data.rover_int_acc(:, 2));
acc_z = filter(b,a,flight_log_data.rover_int_acc(:, 3));

figure; hold on;
plot(flight_log_data.rover_t/1000, acc_x, 'b');
plot(flight_log_data.rover_t/1000, acc_y, 'g');
plot(flight_log_data.rover_t/1000, acc_z, 'r');
legend('Accel Data X', 'Accel Data Y', 'Accel Data Z');
title('Accelerometers Calibration Data');
xlabel('seconds');
ylabel('m/s^2');

% figure; hold on;
% plot(flight_log_data.rover_t, flight_log_data.rover_int_acc(:, 1), 'b');
% plot(flight_log_data.rover_t, acc_x, 'r');
% title(sprintf('X Max: %f, X Min: %f', max(acc_x), min(acc_x)));
% disp(sprintf('X Max: %f, X Min: %f', max(acc_x), min(acc_x)));
% 
% figure; hold on;
% plot(flight_log_data.rover_t, flight_log_data.rover_int_acc(:, 2), 'b');
% plot(flight_log_data.rover_t, acc_y, 'r');
% title(sprintf('Y Max: %f, Y Min: %f', max(acc_y), min(acc_y)));
% disp(sprintf('Y Max: %f, Y Min: %f', max(acc_y), min(acc_y)));
% 
% figure; hold on;
% plot(flight_log_data.rover_t, flight_log_data.rover_int_acc(:, 3), 'b');
% plot(flight_log_data.rover_t, acc_z, 'r');
% 
% title(sprintf('Z Max: %f, Z Min: %f', max(acc_z), min(acc_z)));
% disp(sprintf('Z Max: %f, Z Min: %f', max(acc_z), min(acc_z)));

% acc_len = max(size(acc_x));
% acc_mag = sqrt(acc_x.^2+acc_y.^2+acc_z.^2);
% acc_est = zeros(acc_len, 2); %pitch, roll
% acc_est_2 = zeros(acc_len, 2); %pitch, roll
% acc_vel = zeros(acc_len, 2);
% acc_disp = zeros(acc_len, 2);
% 
% for i = 2:acc_len - 1
% %      acc_est_2(i, 1) = acos(acc_x(i, 1)/acc_mag(i, 1));
% %      acc_est_2(i, 2) = acos(acc_y(i, 1)/acc_mag(i, 1));
%     acc_est(i, 1) = atan2(acc_x(i, 1),sqrt(acc_y(i, 1)^2 + acc_z(i, 1)^2));
%     acc_est(i, 2) = atan2(acc_y(i, 1),sqrt(acc_x(i, 1)^2 + acc_z(i, 1)^2));
%     
%     acc_vel(i, 1) = sum(acc_vel(1:i-1, 1)) + sin(acc_est(i, 1))*acc_x(i, 1);
%     acc_vel(i, 2) = sum(acc_vel(1:i-1, 2)) + sin(acc_est(i, 2))*acc_y(i, 1);
%     
%     acc_disp(i, 1) = sum(acc_disp(1:i-1, 1));
%     acc_disp(i, 2) = sum(acc_disp(1:i-1, 2));
%     
%     %acc_est(i, 3) = acos(acc_z(i, 1)/acc_mag(i, 1));
% end


%acc_disp


% figure; hold on;
% plot(flight_log_data.rover_t/1000, rad2deg(acc_est));
% legend('X angle', 'Y angle');

% figure; hold on;
% plot(flight_log_data.rover_t/1000, rad2deg(acc_est_2));
% legend('X angle', 'Y angle');

% figure;
% plot(flight_log_data.rover_t/1000, acc_vel);
% legend('X speed', 'Y speed');
% %ylim([-10,10]);
% 
% disp_mag = (acc_disp(:,1).^2 + acc_disp(:,2).^2).^0.5;
% figure; hold on;
% plot(flight_log_data.rover_t/1000, acc_disp);
% plot(flight_log_data.rover_t/1000, disp_mag, 'm');
% legend('X disp', 'Y disp', 'XY disp mag');



% You can then calculate the angle between the x-axis and the force vector (R) by using acos (inverse of cosinus):
% accXangle = acos(accXval/R)
% It is the same with the y-axis:
% accYangle = acos(accYval/R)


end
%% gyro cal___
if gyro
[b,a] = butter(9,0.01);
gyro_x = filter(b,a,flight_log_data.rover_ex_gyro(:, 1));
gyro_y = filter(b,a,flight_log_data.rover_ex_gyro(:, 2));
gyro_z = filter(b,a,flight_log_data.rover_ex_gyro(:, 3));

% %flight_log_data.rover_ex_gyro_temp = (flight_log_data.rover_ex_gyro_temp > -100).*flight_log_data.rover_ex_gyro_temp;
% 
% figure;hold on;
% plot(flight_log_data.rover_ex_gyro_temp, gyro_x, '.b')
% %plot(flight_log_data.rover_ex_gyro_temp, ones(size(flight_log_data.rover_ex_gyro_temp)).*mean(gyro_x), 'r')
% %title(sprintf('X Gyro LPF, Mean: %f', mean(gyro_x)));
% %xlim([-20, 50]);
% 
% %figure;hold on;
% plot(flight_log_data.rover_ex_gyro_temp, gyro_y, '.g')
% %plot(flight_log_data.rover_ex_gyro_temp, ones(size(flight_log_data.rover_ex_gyro_temp)).*mean(gyro_y), 'r')
% %title(sprintf('Y Gyro LPF, Mean: %f', mean(gyro_y)));
% %xlim([-20, 50]);
% 
% %figure;hold on;
% plot(flight_log_data.rover_ex_gyro_temp, gyro_z, '.r')
% %plot(flight_log_data.rover_ex_gyro_temp, ones(size(flight_log_data.rover_ex_gyro_temp)).*mean(gyro_z), 'r')
% %title(sprintf('Z Gyro LPF, Mean: %f', mean(gyro_z)));
% title('Gyroscope rotational velocity vs. temperature');
% ylabel('rad/s');
% xlabel('temperture degrees C');
% xlim([-20, 50]);
% legend(sprintf('X Gyro LPF, Mean: %f', mean(gyro_x)), sprintf('Y Gyro LPF, Mean: %f', mean(gyro_y)), sprintf('Z Gyro LPF, Mean: %f', mean(gyro_z)));

figure;hold on;
plot(flight_log_data.rover_t/1000, gyro_x, 'b')
%title(sprintf('X Gyro LPF, Mean: %f', mean(gyro_x)));

%figure;hold on;
plot(flight_log_data.rover_t/1000, gyro_y, 'g')
%title(sprintf('Y Gyro LPF, Mean: %f', mean(gyro_y)));

%figure;hold on;
plot(flight_log_data.rover_t/1000, gyro_z, 'r')

plot(flight_log_data.rover_t/1000, ones(size(flight_log_data.rover_t)).*mean(gyro_x), 'k')
plot(flight_log_data.rover_t/1000, ones(size(flight_log_data.rover_t)).*mean(gyro_y), 'k')


plot(flight_log_data.rover_t/1000, ones(size(flight_log_data.rover_t)).*mean(gyro_z), 'k')
%title(sprintf('Z Gyro LPF, Mean: %f', mean(gyro_z)));
ylabel('rad/s');
xlabel('seconds');
title('Gyroscope rotational velocity vs. time at around room temperature');
legend(sprintf('X Gyro LPF, Mean: %f', mean(gyro_x)), sprintf('Y Gyro LPF, Mean: %f', mean(gyro_y)), sprintf('Z Gyro LPF, Mean: %f', mean(gyro_z)));


end
if est
    
[b_acc,a_acc] = butter(20,0.3);
% acc_x = filter(b_acc,a_acc,flight_log_data.rover_int_acc(:, 1));
% acc_y = filter(b_acc,a_acc,flight_log_data.rover_int_acc(:, 2));
% acc_z = filter(b_acc,a_acc,flight_log_data.rover_int_acc(:, 3));
%acc_full = [acc_x acc_y acc_z];
acc_x = flight_log_data.rover_int_acc(:, 1);
acc_y = flight_log_data.rover_int_acc(:, 2);
acc_z = flight_log_data.rover_int_acc(:, 3);

[b_gyro,a_gyro] = butter(3,0.3);
% gyro_x = filter(b_gyro,a_gyro,flight_log_data.rover_ex_gyro(:, 1));
% gyro_y = filter(b_gyro,a_gyro,flight_log_data.rover_ex_gyro(:, 2));
% gyro_z = filter(b_gyro,a_gyro,flight_log_data.rover_ex_gyro(:, 3));
gyro_x = flight_log_data.rover_ex_gyro(:, 1);
gyro_y = flight_log_data.rover_ex_gyro(:, 2);
gyro_z = flight_log_data.rover_ex_gyro(:, 3);
    
time_d = flight_log_data.rover_t(2:end) - flight_log_data.rover_t(1:end-1);
gyro_len = max(size(flight_log_data.rover_ex_gyro(:, 1)));
gyro_est = zeros(gyro_len-1, 2);
acc_est = zeros(gyro_len-1, 2);
overall_est = zeros(gyro_len-1, 2);

gyro_trust = 0.10;
acc_trust = 1-gyro_trust;

for i = 2:gyro_len - 1
    % C like filter (implemented IIR)
%     gyro_x(i, 1) = lp_filt_c(gyro_x(i-1, 1), flight_log_data.rover_ex_gyro(i, 1), 3);
%     gyro_y(i, 1) = lp_filt_c(gyro_y(i-1, 1), flight_log_data.rover_ex_gyro(i, 2), 3);
%     
%     acc_x(i, 1) = lp_filt_c(acc_x(i-1, 1), flight_log_data.rover_int_acc(i, 1), 9);
%     acc_y(i, 1) = lp_filt_c(acc_y(i-1, 1), flight_log_data.rover_int_acc(i, 2), 9);
%     acc_z(i, 1) = lp_filt_c(acc_z(i-1, 1), flight_log_data.rover_int_acc(i, 3), 9);
    
  gyro_est(i, 1) = (gyro_x(i, 1))*time_d(i)/1000;
  gyro_est(i, 2) = (gyro_y(i, 1))*time_d(i)/1000;
  %gyro_est(i, 3) = overall_est(i-1, 3) + (flight_log_data.rover_ex_gyro(i, 3))*time_d(i)/1000;
    
  acc_est(i, 1) = atan2(acc_x(i, 1),sqrt(acc_y(i, 1)^2 + acc_z(i, 1)^2));
  acc_est(i, 2) = atan2(acc_y(i, 1),sqrt(acc_x(i, 1)^2 + acc_z(i, 1)^2));
  
  overall_est(i, :) = gyro_trust*(overall_est(i-1, :) + gyro_est(i, :)) + acc_est(i, :)*acc_trust;
  
end
figure;
plot(flight_log_data.rover_t(1:end-1)/1000, overall_est);
%check filters
% figure; hold on
% plot(flight_log_data.rover_t/1000, flight_log_data.rover_int_acc-5);
% title('Original');

%figure; hold on
% plot(flight_log_data.rover_t/1000, acc_x, 'b');
% plot(flight_log_data.rover_t/1000, acc_y, 'r');
% plot(flight_log_data.rover_t/1000, acc_z, 'g');
% title('Filtered');

% [b_acc,a_acc] = butter(2,0.2);

%figure; hold on
% plot(flight_log_data.rover_t/1000, filter(b_acc,a_acc,flight_log_data.rover_int_acc)+2);
% title('Buttered');

% figure; hold on;
% %plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(overall_est));
% plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_orient(:, 1)), 'b');
% plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_orient(:, 2)), 'g');
% plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg( overall_est(:, 1)), 'c');
% plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg( overall_est(:, 2)), 'm');
% legend('Roll Est', 'Pitch Est', 'Calc Roll Est', 'Calc Pitch Est');
% title(sprintf('Pitch and Roll Estimates, On-Board'));
% ylabel('degrees');
% xlabel('seconds');

%ang acc est
% three_dt = ones(max(size(flight_log_data.rover_t(1:end-1))), 3);
% three_dt(:, 1) = three_dt(:, 1).*((flight_log_data.rover_t(2:end) - flight_log_data.rover_t(1:end-1))/1000);
% three_dt(:, 2) = three_dt(:, 2).*((flight_log_data.rover_t(2:end) - flight_log_data.rover_t(1:end-1))/1000);
% three_dt(:, 3) = three_dt(:, 3).*((flight_log_data.rover_t(2:end) - flight_log_data.rover_t(1:end-1))/1000);
% ang_acc_est = (flight_log_data.rover_ex_gyro(2:end, :) - flight_log_data.rover_ex_gyro(1:end-1, :))./three_dt;
% 
% %[b,a] = butter(20,0.2);
% %ang_acc_est = filter(b,a,ang_acc_est);
% 
% figure; hold on;
% %plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(ang_acc_est(:, 1)), 'r');
% %plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(ang_acc_est(:, 2)), 'm');
% plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_accs_est(:, 2)), 'b');
% plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_accs_est(:, 1)), 'g');
% %legend('ang acc est calc X(pitch)', 'ang acc est calc Y(roll)', 
% legend('ang acc est onboard X(pitch)', 'ang acc est onboard Y(roll)');
% 
% yaw_est = zeros(gyro_len-1, 1);
% yaw_est_uncorrected = zeros(gyro_len-1, 1);

for i = 2:gyro_len - 1   
%corrected mag vector
pitch = flight_log_data.rover_orient(i, 2);%overall_est(i, 2);
roll = flight_log_data.rover_orient(i, 1);%overall_est(i, 1);

Rp = [-cos(pitch) 0 sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
Rr = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];

corrected_mag = Rr*Rp*[flight_log_data.rover_int_mag(i, 2), flight_log_data.rover_int_mag(i, 1), flight_log_data.rover_int_mag(i, 3)]';
yaw_est(i,1) = atan2(corrected_mag(1,1),corrected_mag(2,1));
yaw_est_uncorrected(i,1) = -atan2(flight_log_data.rover_int_mag(i, 2), flight_log_data.rover_int_mag(i, 1));
%  
end

%correct for mags (RECTIFY)
%yaw_est = yaw_est;
yaw_est_control = yaw_est_uncorrected;
yaw_offset = zeros(max(size(yaw_est)), 1);

for i = 2:max(size(yaw_est))
    diff = rad2deg(yaw_est_control(i, 1) - yaw_est_control(i-1, 1));
    if (diff > 180)
        yaw_offset(i,1) = yaw_offset(i-1,1)-360;
    elseif (diff < -180)
        yaw_offset(i,1) = yaw_offset(i-1 ,1)+360;
    else
        yaw_offset(i,1) = yaw_offset(i-1,1);
    end
    %yaw_est(i, 1) = yaw_est(i, 1) + deg2rad(yaw_offset(i,1));
    yaw_est_uncorrected(i,1) = yaw_est_uncorrected(i,1) +deg2rad(yaw_offset(i,1));
end

yaw_est_control = flight_log_data.rover_orient(:,3);
yaw_est_rectified = flight_log_data.rover_orient(:,3);
yaw_offset = zeros(max(size(yaw_est)), 1);
for i = 2:max(size(yaw_est))
    diff = rad2deg(yaw_est_control(i, 1) - yaw_est_control(i-1, 1));
    if (diff > 180)
        yaw_offset(i,1) = yaw_offset(i-1,1)-360;
    elseif (diff < -180)
        yaw_offset(i,1) = yaw_offset(i-1 ,1)+360;
    else
        yaw_offset(i,1) = yaw_offset(i-1,1);
    end    
    yaw_est_rectified(i,1) = yaw_est_rectified(i,1) +deg2rad(yaw_offset(i,1));
end

%rectify to full coord frame (doesn't really work well)
% yaw_crossedover = flight_log_data.rover_orient(:,3)+((flight_log_data.rover_orient(:,3) > 0).*(flight_log_data.rover_orient(:,3)-pi*3));
% yaw_crossedover_mat = yaw_est+((yaw_est > 0).*(yaw_est-pi*3));


%  [b,a] = butter(9,0.1);
%  figure; hold on;
% % %plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(yaw_crossedover_mat), 'b');
% %plot(flight_log_data.rover_t(1:end-1), rad2deg(yaw_est), 'b');
% % plot(flight_log_data.rover_t(1:end-1), rad2deg(yaw_est_uncorrected), 'r');
%  plot(flight_log_data.rover_t, rad2deg(yaw_est_rectified), 'r');
% %plot(flight_log_data.rover_t(1:end-1), yaw_offset, 'k');
%  legend('Yaw Estimation Corrected for Pitch and Roll');
% % legend('Yaw Estimation Uncorrected');
% title('Corrected Yaw Estimate');
%  ylabel('degrees');
%  xlabel('seconds');

%figure; hold on;
%plot(flight_log_data.rover_t(1:end-1), rad2deg(yaw_est(2:end) - yaw_est(1:end-1)), 'r.');

%dead reckoning with accs after estimate (not a good idea)

raw_acc = zeros(gyro_len-1, 3);

level_frame_acc = zeros(gyro_len-1, 3);
level_frame_vel = zeros(gyro_len-1, 3);
level_frame_pos = zeros(gyro_len-1, 3);


for i = 2:gyro_len - 1  
    pitch = overall_est(i, 1);
    roll = overall_est(i, 2);
    yaw = yaw_est(i, 1);
    
    Rx = [1 0 0; 0 cos(pitch) sin(pitch); 0 -sin(pitch) -cos(pitch)];
    Ry = [cos(roll) 0 -sin(roll); 0 1 0; sin(roll) 0 cos(roll)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0;0 0 1];
    
    level_frame_acc(i,:) = Rx*Ry*Rz*flight_log_data.rover_int_acc(i, :)' + [0 0 9.80665]';
    %raw_acc(i, 1) = flight_log_data.rover_int_acc(i, :)' + [0 0 9.80665]';
    
    level_frame_vel(i,:) = level_frame_vel(i-1,:)+ 0.1*level_frame_acc(i-1,:).*time_d(i)/1000;
    level_frame_pos(i,:) = level_frame_pos(i-1,:)+ level_frame_vel(i-1,:).*time_d(i)/1000;
    
end
end
%  figure;
%  plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(level_frame_acc));
%  legend('Prismatic Acceleration');
%  ylabel('m/s^2');
%  
%  figure;
%  plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(level_frame_vel));
%  legend('Prismatic Velocity');
%  ylabel('m/s');
%  
%  figure;
%  plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(level_frame_pos));
%  legend('Prismatic Position');
%  ylabel('m');
% figure;
% plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_int_acc));


if(pids)
    figure;
    roll_err = flight_log_data.rover_orient_des(:, 1) - flight_log_data.rover_orient(:, 1);
    pitch_err = flight_log_data.rover_orient_des(:, 2) - flight_log_data.rover_orient(:, 2);
    yaw_err = flight_log_data.rover_orient_des(:, 3) - flight_log_data.rover_orient(:, 3);
    
    subplot(3,1,1);hold on;  
    plot(flight_log_data.rover_t/1000,  rad2deg(flight_log_data.rover_orient(:, 1)), 'b');
    plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_orient(:, 2)), 'g');
    plot(flight_log_data.rover_t/1000,  rad2deg(flight_log_data.rover_orient_des(:, 1)), 'r');
    plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_orient_des(:, 2)), 'm');
    %plot(flight_log_data.rover_t/1000, yaw_error, 'r');
    legend('Roll Est', 'Pitch Est', 'Roll Des', 'Pitch Des');
    
    
%     figure;plot(flight_log_data.rover_t/1000, yaw_err);
%     title('non normalized');
    
    %normalize yaw error
    M_PI = pi;
    for i=1:max(size(yaw_err))        
        while (yaw_err(i, 1) > M_PI/2)                          
            yaw_err(i, 1) = yaw_err(i, 1) - M_PI;        
        end                       
        while (yaw_err(i, 1) < -M_PI/2)
            yaw_err(i, 1) = yaw_err(i, 1) + M_PI; 
        end
    end
    
%     figure;plot(flight_log_data.rover_t/1000, yaw_err);
%      title('normalized');
%      [b,a] = butter(2,0.5);
%      figure;plot(flight_log_data.rover_t/1000, filter(b, a, yaw_err));
%      title('normalized');
     
    
%     figure;hold on;
%     plot(flight_log_data.rover_t/1000, rad2deg(roll_err), 'b');
%     plot(flight_log_data.rover_t/1000, rad2deg(pitch_err), 'r');
%     legend('Roll Err', 'Pitch Err');
    
%     figure;hold on;
%     plot(flight_log_data.rover_t/1000, pitch_err, 'b');
%     plot(flight_log_data.rover_t/1000, pitch_err-roll_err, 'g');
%     plot(flight_log_data.rover_t/1000, pitch_err+roll_err, 'r');
%      legend('Pitch Err', 'FL', 'FR');
%      
%      figure;hold on;
%     plot(flight_log_data.rover_t/1000, roll_err, 'b');
%     plot(flight_log_data.rover_t/1000, -pitch_err+roll_err, 'g');
%     plot(flight_log_data.rover_t/1000, pitch_err+roll_err, 'r');
%      legend('Roll Err', 'RR', 'FR');
%     
%     figure;hold on;
%     plot(flight_log_data.rover_t/1000, pitch_err-roll_err, 'b');
%     plot(flight_log_data.rover_t/1000, pitch_err+roll_err, 'g');
%     plot(flight_log_data.rover_t/1000, -pitch_err+roll_err, 'r');
%     plot(flight_log_data.rover_t/1000, -pitch_err-roll_err, 'c');
%     legend('FL Prop', 'FR Prop', 'RR Prop', 'RL Prop');
%     title('theorectical');
    
    subplot(3,1,2);hold on;     
    
%     pitch_rate_error = 0.1*rad2deg(pitch_err(2:end) - pitch_err(1:end-1));
%     roll_rate_error = 0.1*rad2deg(roll_err(2:end) - roll_err(1:end-1));
%     
%     plot(flight_log_data.rover_t(1:end-1)/1000, -pitch_rate_error, 'g');
%     plot(flight_log_data.rover_t(1:end-1)/1000, roll_rate_error, 'b');
%     legend('Delta Pitch Error', 'Delta Roll Error');
%     %xlim([15, 20]);
    
%     subplot(3,1,2); hold on;
    %plot(flight_log_data.rover_t/1000, flight_log_data.rover_ex_gyro(:,1:2));    
    %legend('Gyro Rate X (pitchy axis)', 'Gyro Rate Y (rolly axis)');
    
    
    
  gyro_x = flight_log_data.rover_ex_gyro(:,1);
    gyro_y = flight_log_data.rover_ex_gyro(:,2);
    
%     p_t_delta = -(gyro_x)*0.035;%both conservative
%     r_t_delta = -(gyro_y)*0.025;
%     subplot(4,1,4);hold on;  
%    plot( flight_log_data.rover_t/1000, flight_log_data.rover_m_thrust(:,1) + p_t_delta + r_t_delta, 'b');
%    plot( flight_log_data.rover_t/1000, flight_log_data.rover_m_thrust(:,2) + p_t_delta - r_t_delta, 'g');
%    plot( flight_log_data.rover_t/1000, flight_log_data.rover_m_thrust(:,3) - p_t_delta - r_t_delta, 'r');
%    plot( flight_log_data.rover_t/1000, flight_log_data.rover_m_thrust(:,4) - p_t_delta + r_t_delta, 'c');
%     legend('USELESS FL Prop', 'FR Prop', 'RR Prop', 'RL Prop');


    
    subplot(3,1,2); hold on;
    plot(flight_log_data.rover_t/1000, rad2deg(gyro_x), 'b');
    plot(flight_log_data.rover_t/1000, rad2deg(gyro_y), 'g');
    plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_rates_des(:, 2)), 'r');
    plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_rates_des(:, 1)), 'm');
    legend('Gyro Rate X(Pitch)', 'Gyro Rate Y(Roll)', 'Des Rate Pitch', 'Des Rate Roll');
    %xlim([15, 20]);
    
    
    
    subplot(3,1,3); hold on;
%     plot(flight_log_data.rover_t/1000, rad2deg(abs(flight_log_data.rover_rates_des(:, 1) - gyro_y)));
%     plot(flight_log_data.rover_t/1000, rad2deg(abs(flight_log_data.rover_rates_des(:, 2) - gyro_x)), 'r');
%     legend('abs Des Rate Roll - Gyro Rate Y(Roll)', 'abs Des Rate Pitch - Gyro Rate X(Pitch)');
    
%plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(ang_acc_est(:, 1)), 'r');
%plot(flight_log_data.rover_t(1:end-1)/1000, rad2deg(ang_acc_est(:, 2)), 'm');
plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_accs_est(:, 2)), 'b');
plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_accs_est(:, 1)), 'g');
%legend('ang acc est calc X(pitch)', 'ang acc est calc Y(roll)', 
legend('ang acc est onboard X(pitch)', 'ang acc est onboard Y(roll)');
ylabel('m/s^2');

figure; hold on;
plot(flight_log_data.rover_t/1000, flight_log_data.rover_m_thrust);
    legend('FL Prop', 'FR Prop', 'RR Prop', 'RL Prop');
%     title('actual');

%     
%     %[b,a] = butter(20,0.15);
%     figure; hold on;
%     plot(flight_log_data.rover_t/1000, rad2deg(abs(flight_log_data.rover_orient_des(:, 1) - flight_log_data.rover_orient(:, 1))), 'b');
%     plot(flight_log_data.rover_t/1000, rad2deg(abs(flight_log_data.rover_orient_des(:, 2) - flight_log_data.rover_orient(:, 2))), 'r');
%     
%     legend('roll err abs', 'pitch err abs');
%     
%     figure; hold on;
%     plot(flight_log_data.rover_t/1000, rad2deg(wrapToPi(flight_log_data.rover_orient_des(:, 3))), 'r');
%     plot(flight_log_data.rover_t/1000, rad2deg(wrapToPi(flight_log_data.rover_orient(:, 3)-pi)), 'b');
%     legend('des yaw', 'yaw');
%     
% %     figure; hold on;
% %     plot(flight_log_data.rover_t/1000, rad2deg(wrapToPi(wrapToPi(flight_log_data.rover_orient_des(:, 3)-pi) - wrapToPi(flight_log_data.rover_orient(:, 3)-pi))), 'r');
% %     
% %     legend('yaw err');
%     
%      figure; hold on;
%     plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_rates_des(:, 3)), 'r');
%     plot(flight_log_data.rover_t/1000, rad2deg(flight_log_data.rover_ex_gyro(:,3)), 'b');
%     legend('des yaw rate', 'yaw rate');
% [b,a] = butter(20,0.2);
% %figure;hold on;
% 
% %add step:
% initial_amplitude = deg2rad(180);
% final_amplitude = deg2rad(225); 
% time_point = 57790;
% step_series = ones(max(size(gyro_x)), 1);
% for i = 1:max(size(gyro_x))
%    if flight_log_data.rover_t(i, 1) < time_point
%        step_series(i,1) = step_series(i,1)*initial_amplitude;
%    else
%        step_series(i,1) = step_series(i,1)*final_amplitude;
%    end
% end

%plot(flight_log_data.rover_t/1000,rad2deg(step_series), 'r');
%plot(flight_log_data.rover_t/1000,rad2deg(flight_log_data.rover_orient));

%legend();

end
 
if noise_check
   figure;
   hold on;
   plot(flight_log_data.rover_t/1000,rad2deg(flight_log_data.rover_orient(:,1)), 'b');
   plot(flight_log_data.rover_t/1000,rad2deg(flight_log_data.rover_orient(:,2)), 'r');
   
   legend('Est X', 'Est Y');
    
end


% mag_x = filter(b,a,flight_log_data.rover_int_mag(:, 1));
%%___

% [b,a] = butter(10,0.1);
% dataOut = filter(b,a,flight_log_data.rover_ex_acc(:, 1));
% figure;
% plot(flight_log_data.rover_t, flight_log_data.rover_ex_acc(:, 1));
% hold on;
% plot(flight_log_data.rover_t, -flight_log_data.rover_int_acc(:, 2), 'r');