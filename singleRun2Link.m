function [ returnData ] = singleRun2Link( controller, targ_toe_x,targ_toe_dx,targ_toe_ddx,N,t,kp,kv)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
m = dubPend();
muj = DUBPENDMUJOCO();

DT = t(2)-t(1);

q = zeros(2,N);
qd = zeros(2,N);
qdd = zeros(2,N);
jointTorq = zeros(2,N);
% jointAccel = zeros(2,N);
toePos = zeros(3,N);

q(:,1) = [0;0];
qd(:,1) = [0;0];

muj.set_state(q(:,1),qd(:,1));
 
for i=1:1:(N-1)
    
    rawTau = controller( m, q(:,i), qd(:,i), targ_toe_x(:,i), targ_toe_dx(:,i), targ_toe_ddx(:,i), kp, kv );
    
    jointTorq(:,i) = limit(rawTau, -500, 500);

%     set_state(m, q(:,i), qd(:,i));
%     state = get_state(m);
%     [H,h,~,~,~] = get_dynamic_info(m);
%     
%     qdd(:,i) = H\(jointTorq(:,i) - h);
%     
%     q(:,i+1) = qd(:,i)*DT + q(:,i);
%     qd(:,i+1) = qdd(:,i)*DT + qd(:,i);

    [A, B, H, bias, f_passive] = muj.get_dynamic_info();    
    [H1,h1,~,Jc1,Jc_dot1] = get_dynamic_info(m);
    muj.step(jointTorq(:,i));
    state = muj.get_state();
    
    state.q;
    state.xpos;
    
    q(:,i+1) = state.q;
    qd(:,i+1) = state.qd;
    qdd(:,i+1) = state.qdd;
    
    toePos(:,i+1) = state.xpos;
    
end

m.close();
muj.close();

returnData.q = q;
returnData.dq = qd;
returnData.qdd = qdd;
returnData.tau = jointTorq;
returnData.t = t;
returnData.toePos = toePos;

end

function x = limit(y, lower, upper)

x = min(max(y,lower),upper);

end

