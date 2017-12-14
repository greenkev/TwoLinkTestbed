%% Top Level Script to run simulation and compare two controllers
addpath('./controllers')
N = 10000;
t = (1:N)./2000;
DT = 1/2000;

kp = 100;
kv = sqrt(4*kp); %critical damping

%Generate an elliptical trajectory to follow in the x-y plane with a center,
%x,y radius and angular frequency
omega = 10;
Rx = 0.3;
Ry = 0.3;
xcenter = 1;
ycenter = 1;

targ_toe_x = [xcenter;ycenter;0]*ones(1,length(t)) + [1;0;0]*Rx*(cos(omega*t) - 1) + [0;1;0;]*Ry*sin(omega*t);
targ_toe_dx = zeros(size(targ_toe_x)) - omega*[1;0;0]*Rx*sin(omega*t) + omega*[0;1;0;]*Ry*cos(omega*t);
targ_toe_ddx = zeros(size(targ_toe_x)) - omega^2*[1;0;0]*Rx*cos(omega*t) - omega^2*[0;1;0;]*Ry*sin(omega*t);
 
%Simulate two runs, one with a Task Space Controller, one with a Jacobian
%Transpose controller
tsc = singleRun2Link( @TSC2LinkController, targ_toe_x,targ_toe_dx,targ_toe_ddx,t,kp,kv);
jt = singleRun2Link( @JT2LinkController, targ_toe_x,targ_toe_dx,targ_toe_ddx,t,kp,kv);

%Animate (with video export option) the results of the two simulations
titleStr = 'TSC(red) vs J''(blue)';
animate2Link( t,jt.q,tsc.q,targ_toe_x,titleStr,'NormalExample',0,10,kp,kv)
%% Plots of data
figure(5)
subplot(2,1,1)
hold off
plot(t(1:(end-1)),targ_toe_x(1,1:(end-1)))
hold on
plot(t(1:(end-1)),tsc.toePos(1,1:(end-1)))
plot(t(1:(end-1)),jt.toePos(1,1:(end-1)))
title('Jacobian transpose vs Task Space Control')
xlabel('time (sec)');
ylabel('X end effector position');
legend('Desired','TSC','J''');


subplot(2,1,2)
hold off
plot(t(1:(end-1)),targ_toe_x(2,1:(end-1)))
hold on
plot(t(1:(end-1)),tsc.toePos(2,1:(end-1)))
title('Jacobian transpose vs Task Space Control')
xlabel('time (sec)');
ylabel('Y end effector position');
