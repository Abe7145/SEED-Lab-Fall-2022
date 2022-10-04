% The Seedster s
% PI Motor Controller for Mini project
% created by Scott Reeder 9/28/2022
%

theta = 1; % desired final angle
k = 1; % k value for motor transfer fn
r = 2; % sigma value for motor transfer fn


% open simulation
%
open_system('PIMotorController')

% run the simulation
%
out=sim('PIMotorController');

%plot data
%
figure
plot(out.simout)
