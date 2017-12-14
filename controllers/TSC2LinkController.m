function [ tau_out ] = TSC2LinkController( m, q, qd, desToePos, desToeVel, desToeAcc, kp, kv )
%TSC2LINKCONTROLLER Summary of this function goes here
%   Detailed explanation goes here

velSelect = [zeros(3),eye(3)];
B = eye(2);

set_state(m, q, qd);
state = get_state(m);
[H,h,Jc,Jc_dot] = get_dynamic_info(m);

%Column Vec
pos = state.xpos';
vel = state.xvel';

xdd_toe = desToeAcc + kp*(desToePos - pos) + kv*(desToeVel - velSelect*vel);

qdd_des = pinv(velSelect*Jc,1e-4)*(xdd_toe - velSelect*Jc_dot*qd);

tau_out = pinv(B,1e-4)*(H*qdd_des + h);
end

