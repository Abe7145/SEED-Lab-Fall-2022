% The Seedster 
% PI Forward Position Controller for Demo 1
% Created by Scott Reeder 10/18/2022
%
% Required file: ForwardOuterLoop.slx ForwardOuterLoop.csv
% This program plots the step response of the motor PI velocity controller 
% Pouter value = 0.325975188362578
% Iouter value = 0.00185486950974708
% Pinner value = 0.00152
% Iinner value = 0.0254

theta = 1; % desired final angle
k = 98.917; % k value for motor transfer fn
r = 5.168; % sigma value for motor transfer fn
rwheel = 74;

A = readtable("ForwardOuterLoop.csv"); % open csv of collected motor data
A = table2array(A); % turn the table into an array
i = size(A); % get size of array
i = i(1); % get the number of rows
for j = 1:i
     A(j,1) = A(j,1)/1000; %convert from indecies to seconds
end 

% open simulation
%
open_system('ForwardOuterLoop.slx')

% run the simulation
%
out=sim('ForwardOuterLoop');

%plot data
%
hold on
plot(out.tout,out.simout)
plot(A(:,1),A(:,2))
ylabel("Position (mm)")
xlabel("Time (s)")
%axis([0,4,0,1.3])
hold off
legend("Simulation Data","Experiment Data")
title("Closed Loop Position Control Step Response Expirimental and Simulated Data")
