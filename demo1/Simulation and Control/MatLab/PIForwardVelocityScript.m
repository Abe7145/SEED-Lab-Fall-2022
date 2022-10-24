% The Seedster 
% PI Motor Controller for Demo 1
% Created by Scott Reeder 10/14/2022
%
% Required file: ForwardVelocityStep.slx VelocityPIMotorData.csv
% This program plots the step response of the motor angular velocity PI controller 
% P value = 0.00152
% I value = 0.0254 v/degree*sec

theta = 1; % desired final angle
k = 98.917; % k value for motor transfer fn
r = 5.168; % sigma value for motor transfer fn

A = readtable("VelocityPIMotorData.csv"); % open csv of collected motor data
A = table2array(A); % turn the table into an array
i = size(A); % get size of array
i = i(1); % get the number of rows
for j = 1:i
     A(j,1) = A(j,1)/1000; %convert from indecies to seconds 
end 

% open simulation
%
open_system('ForwardVelocityStep.slx')

% run the simulation
%
out=sim('ForwardVelocityStep');

%plot data
%
hold on
plot(out.tout,out.simout)
plot(A(:,1),A(:,2))
ylabel("Velocity") %rad/sec??
xlabel("Time (s)")
axis([0,4,0,1.3])
hold off
legend("Simulation Data","Experiment Data")
title("Inner loop Velocity step response")
