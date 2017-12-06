
% loadlibrary('dubPend.dll','dubPend.h');
% 
% calllib('dubPend','init');
% 
% unloadlibrary('dubPend');

circ = 0:0.1:2*pi/3;
rbdlObj = dubPend();
q = [1;1];
qd = [1;1];
x1 = [];
x2 = [];

set_state(rbdlObj, q, qd);
state = get_state(rbdlObj)
[H,h1,h2,Jc,JcDot] = get_dynamic_info(rbdlObj)

rbdlObj.close();