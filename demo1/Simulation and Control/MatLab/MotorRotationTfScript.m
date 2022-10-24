% The Seedster 
% Robot forward transfer function
% created by Scott Reeder 9/28/2022
%
% Required file: rotationStepResponse.csv, RotateStepResponse.slx
% This program imports real data from a csv and then plots the data along
% with the calculated transfer function model output 
% assume counter clockwise postive rotation of the robot (like a unit
% circle)
%

k = .88216; % Proportional K term
k3 = 0;
r = 6.36525; % sigma term 14.93
rwheel = 74; %radius of wheel in mm
A = readtable("rotationStepResponse.csv"); % open csv of collected motor data
A = table2array(A); % turn the table into an array
c = 4.444;%number of motor counts per degree
thetaToMm = pi*rwheel/180;
rRobot = 173;

 file = 'RotateStepResponse';
 open_system(file) % open the simulink file for the transfer fn of motor
 out=sim(file);% get the simulink output
 B=A; % create a copy of the data

i = size(A); % get size of array
i = i(1); % get the number of rows
for j = 1:i
     A(j,1) = A(j,1)/1000; %convert from milliseconds to seconds 
     A(j,2) = A(j,2)*thetaToMm/c;%convert from counts to degrees
     A(j,3) = A(j,3)*thetaToMm/c;%convert from counts to degrees now in mm
     C(j,2) = k; % make C a constant value of k
     %C(j,3) = k3;
end 

C=B; % create an array to show the constant final value
for j = 2:i % for every element in the array
    B(j,2) = (A(j,2)-A(j-1,2))/(A(j,1)-A(j-1,1)); %find dtheta/dt (velocity) mm/s
    B(j,3) = (A(j,3)-A(j-1,3))/(A(j,1)-A(j-1,1)); %find dtheta/dt (velocity) mm/s
    C(j,3) = (B(j,3)-B(j,2))/(rRobot);
end


for j = 1:i
     %B(j,1) = B(j,1)/1000; %convert from indecies to seconds 
     B(j,2) = B(j,2);
     C(j,2) = k; % make C a constant value of k
    % C(j,3) = k3;
end 

hold on
%subplot(1,2,1);
%plot(out.tout,out.TF) % plot the motor transfer function model data
%plot(B(:,1),B(:,2)); % plot the real motor step response data
%plot(B(:,1),C(:,2)) % plot the constant final value

%subplot(1,2,2);
plot(out.tout,out.TF)
plot(A(:,1),C(:,2)); % plot the real motor step response data
plot(A(:,1),C(:,3)) % plot the constant final value

axis([0,4.000,0,1.2]) % scale axis

% set axis titles
xlabel("Time (s)")
ylabel("Angular Velocity (rad/second)")
title("Real and modeled motor rotation step response")
legend("Simulated Data","Final Value","Experimental Data")
hold off
