% The Seedster 
% PI Motor Controller for Demo 1
% Created by Scott Reeder 10/14/2022
%
% Required file: RotateVelocityStep.slx motorRotateStepResponse.csv
% This program plots the step response of the motor angular velocity PI controller 
% P value = 0.537324556448308
% I value = 4.00434014925818

theta = 1; % desired final angle
k = .88216; % k value for motor transfer fn
r = 6.36525; % sigma value for motor transfer fn

A = readtable("motorRotateStepResponse.csv"); % open csv of collected motor data
A = table2array(A); % turn the table into an array
i = size(A); % get size of array
i = i(1); % get the number of rows
for j = 1:i
     A(j,1) = A(j,1)/1000; %convert from indecies to seconds 
end 

%open simulation

open_system('RotateVelocityStep.slx')

% run the simulation
%
out=sim('RotateVelocityStep.slx');

%plot data
%

hold on
plot(out.tout, out.simout)
plot(A(:,1),A(:,2))
title("Inner loop Rotational Velocity step response")
ylabel("Velocity (rad/s)")
xlabel("Time (s)")
axis([0,4,0,1.4])
legend("Simulation Data","Experiment Data")
