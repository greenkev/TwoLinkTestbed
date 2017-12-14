classdef DUBPENDMUJOCO < handle
% DUBPENDMUJOCO This class encapsulates the interface with the double
% pendulum MuJoCo model. If usage crashes matlab check the MUJOCO_LOG.TXT
% for more information. If the functions or data structures in 
% src/dubPendMujoco.h are modified, the member properties, blank generator 
% functions and the callLib functions must be modified to match.
    
properties
    %Must match constants in the dubPendMujoco.h
    nQ = 2;
    nU = 2;
    DOF = 3;
    XDD_TARGETS = 1;
    %dt not directly used, but is useful to have match the time step size
    %in the xml model file.
    dt = 0.0005;
    %Structures that match the structs in header file
    state 
    dyn_info
    %Library name as a string for future cross platform support
    libName
    
end

methods

    %Construct the object with the current directory as the location of
    %required files
    function obj = DUBPENDMUJOCO()

        %Libraries likely to be named differently on OSX and Linux
        if ispc()
            obj.libName = 'dubPendMujoco';
        else
            error('Linux and OSX not implemented in DUBPENDMUJOCO. Please fix library name parameters in constructor');
        end

        %Avoid errors incase the library is not unloaded
        if libisloaded(obj.libName)
            unloadlibrary(obj.libName)
        end

        % Check to make sure that the current directory contains
        % mjkey.txt, dubPend.xml, the dynamic library and the header file
        % This is to prevent or to give useful feedback on missing files
        confirmFileExists('mjkey.txt');
        confirmFileExists('src/dubPendMujoco.h');
        confirmFileExists('dubPend.xml');
        confirmFileExists(obj.libName);
 
        loadlibrary(obj.libName,'src/dubPendMujoco.h');

        calllib(obj.libName,'init','.');

        obj.state = obj.blank_state();
        obj.dyn_info = obj.blank_dyn_info();

    end

    %Retrieve useful dynamic matricies and vectors for the current state
    function [A, B, H, bias, f_passive] = get_dynamic_info(obj)
        p_dyn_info = libpointer('dyn_info_t_muj', obj.dyn_info);

        calllib(obj.libName,'get_dynamic_info',p_dyn_info);
        temp = p_dyn_info.Value;

        %Jacobian to target point (end effector)
        A = reshape(temp.A, obj.nQ, [])';

        % selector matrix (maps motors to generalized coordinates). For the
        %double pendulum it is 2x2 identity because every joint is actuated.
        %In general it doesnt have to be
        B = reshape(temp.B, [], obj.nU);

        %Mass matrix
        H = reshape(temp.H, obj.nQ, obj.nQ);

        %bias: gravitational and velocity product terms
        bias = reshape(temp.qfrc_bias, 1, [])';
        %Passive forces (ie springs, damping, ect.)
        f_passive = reshape(temp.qfrc_passive, 1, [])';

    end


    function set_state(obj, q, qd)
    %set_state Set the current internal state of the mujoco simulation to q
    %and the current state velocity to q dot
    
       %Check for correct number of elements in q. 
       %Reshape corrects for row vs collumn vector notation
       assert( all(size(reshape(q,1,[])) == [1,obj.nQ]),'In DUBPENDMUJOCO, set_state(...): q is not the correct size');
       assert( all(size(reshape(qd,1,[])) == [1,obj.nQ]),'In DUBPENDMUJOCO, set_state(...): qd is not the correct size');
       
       calllib(obj.libName,'set_state',reshape(q,1,[]),reshape(qd,1,[])); 
    end

    function state = get_state(obj)
    %get_state() Get current state information (and global state of end 
    %effector) and return it is a structure
        state = libpointer('state_t_muj', obj.state);
        calllib(obj.libName, 'get_state', state);
        state = state.Value;
        state.xpos = reshape(state.xpos, 3, [])'; %Position (x,y,z) of
        state.xvel = reshape(state.xvel, 6, [])'; %6-D Velocity in world frame at site positions
        state.xacc = reshape(state.xacc, 6, [])'; %6-D Accel in world frame at site positions
        %flip rot and lin
        state.xvel = [state.xvel(:,4:6), state.xvel(:,1:3)]; %change to rotational first, then linear
        state.xacc = [state.xacc(:,4:6), state.xacc(:,1:3)];
    end

    function step(obj, u)
    %step() Run the mujoco simulation one step forward with motor torque
    %commands u
       assert( all(size(reshape(u,1,[])) == [1,obj.nU]),'In DUBPENDMUJOCO, step(...): u is not the correct size');
       
       calllib(obj.libName,'step',reshape(u,1,[])); 
    end

    function dyn_info = blank_dyn_info(obj)
    %Get a blank structure with the correct members and sizes
        dyn_info.A = zeros(1, obj.nQ*obj.DOF*2*obj.XDD_TARGETS); %Jacobian to target point
        dyn_info.B = zeros(1, obj.nQ*obj.nU);
        dyn_info.H = zeros(1, obj.nQ*obj.nQ);
        dyn_info.qfrc_bias = zeros(1, obj.nQ);
        dyn_info.qfrc_passive = zeros(1, obj.nQ);            
    end

    function state = blank_state(obj) 
    %Get a blank structure with the correct members and sizes           
        state.q = zeros(1, obj.nQ);
        state.qd = zeros(1, obj.nQ);
        state.qdd = zeros(1, obj.nQ);
        state.xpos = zeros(1, obj.DOF*obj.XDD_TARGETS);
        state.xvel = zeros(1, obj.DOF*obj.XDD_TARGETS*2);
        state.xacc = zeros(1, obj.DOF*obj.XDD_TARGETS*2);
    end

    function close(obj) 
        %Clear internal states and info structures
        obj.state = [];
        obj.dyn_info = [];
        %Unload the library, this allows for recompilation between
        %executions
        unloadlibrary(obj.libName);        
    end
end

end


function confirmFileExists(fileName)
%Check if the input file, exists. If it doesnt, throw a helpful error
    if exist(['./',fileName],'file') ~= 2
        error(['Required file missing: ',fileName],' This could also be caused by calling the constructor from the wrong directory');
    end
end

