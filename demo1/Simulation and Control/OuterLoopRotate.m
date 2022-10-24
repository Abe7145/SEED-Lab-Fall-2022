% The Seedster 
% PI Rotation Controller for Demo 1
% Created by Scott Reeder 10/18/2022
%
% Required file: RotateOuterLoop.slx RotateOuterLoop.csv
% This program plots the step response of the motor PI rotational
% controller
% Pouter value = 8.506
% Iouter value = 1.272
% Pinner value = 0.537324556448308
% Iinner value = 4.00434014925818

theta = 1; % desired final angle
k = 98.917; % k value for motor transfer fn
r = 5.168; % sigma value for motor transfer fn
rwheel = 74;

A = readtable("RotateOuterLoop.csv"); % open csv of collected motor data
A = table2array(A); % turn the table into an array
i = size(A); % get size of array
i = i(1); % get the number of rows
for j = 1:i
     A(j,1) = A(j,1)/1000; %convert from indecies to seconds
end 

% open simulation
%
open_system('RotateOuterLoop.slx')

% run the simulation
%
out=sim('RotateOuterLoop');

%plot data
%
hold on
plot(out.tout,out.simout)
plot(A(:,1),A(:,2))
ylabel("Position (degrees)")
xlabel("Time (s)")
%axis([0,4,0,1.3])
hold off
legend("Simulation Data","Experiment Data")
title("Closed Loop Angle Control Step Response Expirimental and Simulated Data")
