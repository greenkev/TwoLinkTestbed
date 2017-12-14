function [ returnData ] = singleRun2Link( controller, targ_toe_x,targ_toe_dx,targ_toe_ddx,t,kp,kv)
%SINGLERUN2LINK This function runs one trial for the double pendulum with
%the specified controller. It assumes on the controller passed in taking the
%correct input parameters and that the target pos,vel and accel have the same
%size as t.

%Load and initialize the MuJoCo and RBDL libraries
m = dubPend();
muj = DUBPENDMUJOCO();

DT = t(2)-t(1);
N = length(t);

%Recorded State and input Information
q = zeros(2,N);
qd = zeros(2,N);
qdd = zeros(2,N);
jointTorq = zeros(2,N);
toePos = zeros(3,N);

%Position Initialization
q(:,1) = [0;0];
qd(:,1) = [0;0];
muj.set_state(q(:,1),qd(:,1));
 
for i=1:1:(N-1)
    
    %Run Controller
    rawTau = controller( m, q(:,i), qd(:,i), targ_toe_x(:,i), targ_toe_dx(:,i), targ_toe_ddx(:,i), kp, kv );
    
    %Limit the applied torque
    jointTorq(:,i) = limit(rawTau, -500, 500);

    %Step the simulation
    muj.step(jointTorq(:,i));
    
    %Record the state
    state = muj.get_state();    
    
    q(:,i+1) = state.q;
    qd(:,i+1) = state.qd;
    qdd(:,i+1) = state.qdd;    
    toePos(:,i+1) = state.xpos;
    
end

%Unload the libraries
m.close();
muj.close();

%Store all return data into a structure
returnData.q = q;
returnData.dq = qd;
returnData.qdd = qdd;
returnData.tau = jointTorq;
returnData.t = t;
returnData.toePos = toePos;

end

%Simplifies the simulation code
function x = limit(y, lower, upper)

x = min(max(y,lower),upper);

end

