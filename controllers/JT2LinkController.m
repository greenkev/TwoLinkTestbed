function [ tau_out ] = JT2LinkController( m, q, qd, desToePos, desToeVel, desToeAcc, kp, kv )
%TSC2LINKCONTROLLER Summary of this function goes here
%   Detailed explanation goes here

velSelect = [zeros(3),eye(3)];
% B = eye(2);

set_state(m, q, qd);
state = get_state(m);
[~,~,~,Jc,~] = get_dynamic_info(m);

%Column Vec
pos = state.xpos';
vel = state.xvel';

f_toe = kp*(desToePos - pos) + kv*(desToeVel - velSelect*vel);



% qdd_des = pinv(velSelect*Jc)*(xdd_toe - velSelect*Jc_dot*qd);
tau_out = (velSelect*Jc)'*f_toe;
end

