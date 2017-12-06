clear all
clear mex
close all
addpath('controllers')
x = waitbar(0,'Initializing');
m = CASSIE();

state = m.get_state();

Kp = 2000;
Kv = 200;
Kpf = 6000;
Kvf = 200;

N = 5000;
t = (1:N)./2000;

omega = 10;
R1 = 0.1;
R2 = 0.1;

targ_toe_x = [0;0;-0.65]*ones(1,length(t)) + [1;0;0]*R1*(cos(omega*t) - 1) + [0;0;1;]*R2*sin(omega*t);
targ_toe_dx = zeros(size(targ_toe_x)) - omega*[1;0;0]*R1*sin(omega*t) + omega*[0;0;1;]*R2*cos(omega*t);
targ_toe_ddx = zeros(size(targ_toe_x))- omega^2*[1;0;0]*R1*cos(omega*t) - omega^2*[0;0;1;]*R2*sin(omega*t);

qddDesVec = [];
tauVec=[];
for iter=1:1:N
    
    waitbar(iter/N,x,['running: ',num2str(iter),'/',num2str(N)]);
    
%     tau = simpleJTaccel( m, toe_cmd);
%     tau = PDJoint(m,hip,knee);    

%     tau = simpleJT_mex( state.q(2),state.q(3),state.qd(2),state.qd(3), targ_toe_x(:,iter),targ_toe_dx(:,iter),Kpf,Kvf);
%     tau = simpleJT( state.q(2),state.q(3),state.qd(2),state.qd(3), targ_toe_x(:,iter),targ_toe_dx(:,iter),Kpf,Kvf);

    [tau,qdd_des,xdd_toe] = TSC_traj( state.q(2),state.q(3),state.qd(2),state.qd(3), targ_toe_x(:,iter),targ_toe_dx(:,iter),targ_toe_ddx(:,iter),Kp,Kv);
%     [tau,qdd_des] = TSC_traj_mex( state.q(2),state.q(3),state.qd(2),state.qd(3), targ_toe_x(:,iter),targ_toe_dx(:,iter),targ_toe_ddx(:,iter),Kp,Kv);

    
    qfrc = zeros(1,32);
    m.step(tau, qfrc);
    state = m.get_state();
    
    tauVec = [tauVec,tau];
    qddDesVec = [qddDesVec,qdd_des];
    
    log(iter,:) = [state.q, tau',state.xpos(2,1:3)-state.xpos(1,1:3),state.xvel(2,1:3),state.xacc(2,1:3),xdd_toe'];
    
end
close(x);

i_q = 1:13;
i_tau = 14:16;
i_xtoe = 17:19;
i_dxtoe = 20:22;
i_ddxtoe = 23:25;
i_ddx_cmd = 26:27;

figure(6)
% subplot(2,1,1)
hold off
plot(t,qddDesVec(1,:));
hold on
plot(t,qddDesVec(2,:));
% title('Toe Z pos; Jacobian transpose Tracking');
% title('Toe Z pos; TSC Tracking');
% legend('Goal','Simulation');
% xlabel('time (sec)');
% ylabel('Position (m)');
% grid on

figure(2);
subplot(2,1,1)
hold off
plot(t,targ_toe_x(3,:));
hold on
plot(t,log(:,i_xtoe(3)));
% title('Toe Z pos; Jacobian transpose Tracking');
title('Toe Z pos; TSC Tracking');
legend('Goal','Simulation');
xlabel('time (sec)');
ylabel('Position (m)');
grid on

subplot(2,1,2)
hold off
plot(t,targ_toe_x(1,:));
hold on
plot(t,log(:,i_xtoe(1)));
title('Toe X pos');
legend('Goal','Simulation');
xlabel('time (sec)');
ylabel('Position (m)');
set(gcf,'position',[-925 188 841 832])
grid on

figure(3)
% hold off
plot(targ_toe_x(1,:),targ_toe_x(3,:));
hold on
plot(log(:,i_xtoe(1)),log(:,i_xtoe(3)));
title('TSC Tracking');
% title('Jacobian Transpose Tracking');
xlabel('X position (m)');
ylabel('Y position (m)');
legend('Goal','Simulation');
grid on
axis equal

figure(4)
% hold off
plot(t,log(:,i_tau(1)));
hold on
plot(t,log(:,i_tau(2)));
title('Joint Torques, TSC');
legend('Hip','Knee');
grid on

figure(81)
hold off
plot(t,log(:,i_ddx_cmd(2)));
hold on
plot(t,log(:,i_ddxtoe(3)));
title('TSC Tracking');
% title('Jacobian Transpose Tracking');
xlabel('time');
ylabel('Y accel (m/s2)');
legend('Command','Simulation');
grid on

figure(82)
hold off
plot(t,log(:,i_ddx_cmd(1)));
hold on
plot(t,log(:,i_ddxtoe(1)));
title('TSC Tracking');
% title('Jacobian Transpose Tracking');
xlabel('time');
ylabel('X accel (m/s2)');
legend('Command','Simulation');
grid on

m.close();
playback( log(1:25:end,:));



