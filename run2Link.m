addpath('./controllers')
N = 2000;
t = (1:N)./2000;
DT = 1/2000;

omega = 10;
R1 = 0.3;
R2 = 0.3;

kp = 100;
kv = sqrt(4*kp); %critical damping


targ_toe_x = [1;1;0]*ones(1,length(t)) + [1;0;0]*R1*(cos(omega*t) - 1) + [0;1;0;]*R2*sin(omega*t);
targ_toe_dx = zeros(size(targ_toe_x)) - omega*[1;0;0]*R1*sin(omega*t) + omega*[0;1;0;]*R2*cos(omega*t);
targ_toe_ddx = zeros(size(targ_toe_x))- omega^2*[1;0;0]*R1*cos(omega*t) - omega^2*[0;1;0;]*R2*sin(omega*t);
 

tsc = singleRun2Link( @TSC2LinkController, targ_toe_x,targ_toe_dx,targ_toe_ddx,N,t,kp,kv);
jt = singleRun2Link( @JT2LinkController, targ_toe_x,targ_toe_dx,targ_toe_ddx,N,t,kp,kv);

titleStr = 'TSC(red) vs J''(blue)';

animate2Link( t,jt.q,tsc.q,targ_toe_x,titleStr,'NormalExample',0,3,kp,kv)
%%
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
% plot(t(1:(end-1)),jt.toePos(2,1:(end-1)))
title('Jacobian transpose vs Task Space Control')
xlabel('time (sec)');
ylabel('Y end effector position');
