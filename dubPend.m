classdef dubPend < handle
    
    properties
        nQ = 2;
        nC = 1;
        nU = 2;
        DOF = 3;
        dt = 0.0005;
        dyn_info
        state 
        libName
    end
    
    methods
        
        function obj = dubPend()
            
            if ispc()
                obj.libName = 'dubPend';
            else
                obj.libName = 'libdubPend';
            end
            
            if libisloaded(obj.libName)
                unloadlibrary(obj.libName)
            end
            
            if ispc()                
                loadlibrary([obj.libName,'.dll'],'src/dubPend.h');
            else                
                loadlibrary(obj.libName,'dubPend.h');
            end
            
            calllib(obj.libName,'init');
            
            obj.dyn_info = obj.blank_dyn_info();
            obj.state = obj.blank_state();
            
        end
        
        function [H,h1,h2,Jc,JcDot] = get_dynamic_info(obj)
            p_dyn_info = libpointer('dyn_info_t', obj.dyn_info);
            
            calllib(obj.libName,'get_dynamic_info',p_dyn_info);
            temp = p_dyn_info.Value;
                        
            %Mass matrix
            H = reshape(temp.H, obj.nQ, obj.nQ);
            
            %dynamic and static forces 
            h1 = reshape(temp.h1, 1, [])';
            h2 = reshape(temp.h2, 1, [])';
            
            %contact jacobian
            Jc = reshape(temp.Jc, obj.nQ, [])';
            JcDot = reshape(temp.JcDot, obj.nQ, [])';
           
            
        end
        
        function set_state(obj, q, qd)
           calllib(obj.libName,'set_state',reshape(q,1,[]),reshape(qd,1,[])); 
        end
        
        function state = get_state(obj)           
            state = libpointer('state_t', obj.state);
            calllib(obj.libName, 'get_state', state);
            state = state.Value;
            state.xpos = reshape(state.xpos, obj.DOF, [])'; %Position (x,y,z) of
            state.xvel = reshape(state.xvel, obj.DOF*2, [])'; %6-D Velocity in world frame at site positions
        end
        
%         function step(obj, u, qfrc)
%            calllib(obj.libName,'step',reshape(u,1,[]), reshape(qfrc,1,[])); 
%         end
        
        function dyn_info = blank_dyn_info(obj)           
            dyn_info.H = zeros(1, obj.nQ*obj.nQ);
            dyn_info.h1 = zeros(1, obj.nQ);
            dyn_info.h2 = zeros(1, obj.nQ);
            dyn_info.Jc = zeros(1, obj.nQ*obj.DOF*2); %Contact jacobians
            dyn_info.JcDot = zeros(1, obj.nQ*obj.nC*obj.DOF*2); %Contact jacobians
            
        end
        
        function state = blank_state(obj)            
            state.q = zeros(1, obj.nQ);
            state.qd = zeros(1, obj.nQ);
            state.xpos = zeros(1, obj.DOF);
            state.xvel = zeros(1, obj.DOF*2);
        end
        
        function close(obj)            
            obj.dyn_info = [];
            obj.state = [];
            unloadlibrary(obj.libName);
        end
    end
    
end

