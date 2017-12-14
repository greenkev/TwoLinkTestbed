classdef DUBPENDRBDL < handle
% DUBPENDRBDL This class encapsulates the interface with the double
% pendulum RBDL model. If the functions or data structures in 
% src/dubPendRBDL.h are modified, the member properties, blank generator 
% functions and the callLib functions must be modified to match.

properties
    %Must match constants in the dubPendRBDL.h
    nQ = 2;
    nC = 1;
    nU = 2;
    DOF = 3;
    %Structures that match the structs in header file        
    dyn_info
    state 
    %Library name as a string for future cross platform support
    libName
end

methods
    
    %Construct the object with the current directory as the location of
    %required files
    function obj = DUBPENDRBDL()

        if ispc()
            obj.libName = 'dubPendRBDL';
        else
            error('Linux and OSX not implemented in DUBPENDRBDL. Please fix library name parameters in constructor');
        end

        %Avoid errors incase the library is not unloaded
        if libisloaded(obj.libName)
            unloadlibrary(obj.libName)
        end

        % Check to make sure that the current directory contains
        % mjkey.txt, dubPend.xml, the dynamic library and the header file
        % This is to prevent or to give useful feedback on missing files
        confirmFileExists('src/dubPendRBDL.h');
        confirmFileExists(obj.libName);
                      
        loadlibrary([obj.libName,'.dll'],'src/dubPendRBDL.h');

        calllib(obj.libName,'init');

        obj.state = obj.blank_state();
        obj.dyn_info = obj.blank_dyn_info();

    end

    %Retrieve useful dynamic matricies and vectors for the current state
    function [H,h,Jc,JcDot] = get_dynamic_info(obj)
        p_dyn_info = libpointer('dyn_info_t', obj.dyn_info);

        calllib(obj.libName,'get_dynamic_info',p_dyn_info);
        temp = p_dyn_info.Value;

        %Mass matrix
        H = reshape(temp.H, obj.nQ, obj.nQ);

        %gravitational and velocity product terms
        h = reshape(temp.h, 1, [])';

        %End Effector jacobian
        Jc = reshape(temp.Jc, obj.nQ, [])';
        JcDot = reshape(temp.JcDot, obj.nQ, [])';


    end

    function set_state(obj, q, qd)
    %set_state Set the current internal state of the RBDL model to q
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
        state = libpointer('state_t', obj.state);
        calllib(obj.libName, 'get_state', state);
        state = state.Value;
        state.xpos = reshape(state.xpos, obj.DOF, [])'; %Position (x,y,z) of
        state.xvel = reshape(state.xvel, obj.DOF*2, [])'; %6-D Velocity in world frame at site positions
    end

    function dyn_info = blank_dyn_info(obj)
    %Get a blank structure with the correct members and sizes            
        dyn_info.H = zeros(1, obj.nQ*obj.nQ);
        dyn_info.h = zeros(1, obj.nQ);
        dyn_info.Jc = zeros(1, obj.nQ*obj.DOF*2); %Contact jacobians
        dyn_info.JcDot = zeros(1, obj.nQ*obj.nC*obj.DOF*2); %Contact jacobians

    end

    function state = blank_state(obj)
    %Get a blank structure with the correct members and sizes             
        state.q = zeros(1, obj.nQ);
        state.qd = zeros(1, obj.nQ);
        state.xpos = zeros(1, obj.DOF);
        state.xvel = zeros(1, obj.DOF*2);
    end

    function close(obj) 
        %Clear internal states and info structures           
        obj.dyn_info = [];
        obj.state = [];
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
