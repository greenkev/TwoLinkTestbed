classdef DUBPENDMUJOCO < handle
    
    properties
        nQ = 2;
        nU = 2;
        DOF = 3;
        XDD_TARGETS = 1;
        dt = 0.0005;
        state 
        libName
        dyn_info
    end
    
    methods
        
        function obj = DUBPENDMUJOCO()
            
            if ispc()
                obj.libName = 'dubPendMujoco';
            else
                obj.libName = 'libdubpendmujoco';
            end
            
            if libisloaded(obj.libName)
                unloadlibrary(obj.libName)
            end
            loadlibrary(obj.libName,'src/dubPendMujoco.h');
            
            calllib(obj.libName,'init','.');
            
            obj.state = obj.blank_state();
            obj.dyn_info = obj.blank_dyn_info();
            
        end
        
        function [A, B, H, bias, f_passive] = get_dynamic_info(obj)
            p_dyn_info = libpointer('dyn_info_t_muj', obj.dyn_info);
            
            calllib(obj.libName,'get_dynamic_info',p_dyn_info);
            temp = p_dyn_info.Value;
            
            %Jacobian to target point
            A = reshape(temp.A, obj.nQ, [])';
            
            %selector matrix
            B = reshape(temp.B, [], obj.nU);
            
            %Mass matrix
            H = reshape(temp.H, obj.nQ, obj.nQ);
            
            %bias 
            bias = reshape(temp.qfrc_bias, 1, [])';
            f_passive = reshape(temp.qfrc_passive, 1, [])';
            
        end
        
        function set_state(obj, q, qd)
           calllib(obj.libName,'set_state',reshape(q,1,[]),reshape(qd,1,[])); 
        end
        
        function state = get_state(obj)
           
            state = libpointer('state_t_muj', obj.state);
            calllib(obj.libName, 'get_state', state);
            state = state.Value;
            state.xpos = reshape(state.xpos, 3, [])'; %Position (x,y,z) of
            state.xvel = reshape(state.xvel, 6, [])'; %6-D Velocity in world frame at site positions
            state.xacc = reshape(state.xacc, 6, [])'; %6-D Accel in world frame at site positions
            %flip rot and lin
            state.xvel = [state.xvel(:,4:6), state.xvel(:,1:3)]; %change to rotational first, then linear I think ?
            state.xacc = [state.xacc(:,4:6), state.xacc(:,1:3)];
        end
        
        function step(obj, u)
           calllib(obj.libName,'step',reshape(u,1,[])); 
        end
        
        function dyn_info = blank_dyn_info(obj)
          
            dyn_info.A = zeros(1, obj.nQ*obj.DOF*2*obj.XDD_TARGETS); %Jacobian to target point
            dyn_info.B = zeros(1, obj.nQ*obj.nU);
            dyn_info.H = zeros(1, obj.nQ*obj.nQ);
            dyn_info.qfrc_bias = zeros(1, obj.nQ);
            dyn_info.qfrc_passive = zeros(1, obj.nQ);            
        end
        
        function state = blank_state(obj)            
            state.q = zeros(1, obj.nQ);
            state.qd = zeros(1, obj.nQ);
            state.qdd = zeros(1, obj.nQ);
            state.xpos = zeros(1, obj.DOF*obj.XDD_TARGETS);
            state.xvel = zeros(1, obj.DOF*obj.XDD_TARGETS*2);
            state.xacc = zeros(1, obj.DOF*obj.XDD_TARGETS*2);
        end
        
%         function [lb, ub] = get_motor_limits(obj)
%             lims.lb = zeros(1, obj.nU);
%             lims.ub = zeros(1, obj.nU);
%             lims = libpointer('motor_limits_t', lims);
%             calllib(obj.libName,'get_motor_limits',lims);
%             lb = lims.Value.lb;
%             ub = lims.Value.ub;
%         end
        
        function close(obj)      
            obj.state = [];
            unloadlibrary(obj.libName);
        end
    end
    
end

