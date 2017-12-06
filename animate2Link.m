function animate2Link( t,qJT,qTSC,targ_toe_x,titleStr,videoName,exportEnabled,frameSkip,kp,kv)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
L1 = 1;
L2 = 1;
fig = figure(153);
clf(fig);
fig.Position = [-1076 140 880 792];


hold off
target = plot(targ_toe_x(1,1),targ_toe_x(2,1),'ko','markerfacecolor','k');
hold on
arm1 = line([0,1,2],[0,-1,0],'color','b');
arm2 = line([0,1,2],[0,-1,0],'color','r');
plot(targ_toe_x(1,:),targ_toe_x(2,:),'g');
title(titleStr);
t = text(-0.4,1.5,['kp = ',num2str(kp),', kv = ',num2str(kv)]);
axis([-0.3,1.2,-0.3,1.6])
grid on
axis equal


    
% legend('test','test','test')

% legend data;
% 
% set(gca,'LegendColorbarListeners',[]); 
% setappdata(gca,'LegendColorbarManualSpace',1);
% setappdata(gca,'LegendColorbarReclaimSpace',1);

if exportEnabled ~= 0    
    v = VideoWriter(['media/',videoName,'.mp4'],'MPEG-4');
    v.Quality = 100;
    v.FrameRate = 60;
    open(v);
end


for i = 1:frameSkip:length(qJT)
    arm1.XData = [0, L1*cos(qJT(1,i)), L1*cos(qJT(1,i)) + L2*cos(qJT(1,i)+qJT(2,i))];
    arm1.YData = [0, L1*sin(qJT(1,i)), L1*sin(qJT(1,i)) + L2*sin(qJT(1,i)+qJT(2,i))];
    
    arm2.XData = [0, L1*cos(qTSC(1,i)), L1*cos(qTSC(1,i)) + L2*cos(qTSC(1,i)+qTSC(2,i))];
    arm2.YData = [0, L1*sin(qTSC(1,i)), L1*sin(qTSC(1,i)) + L2*sin(qTSC(1,i)+qTSC(2,i))];  
    
    target.XData = targ_toe_x(1,i);
    target.YData = targ_toe_x(2,i);
    drawnow();
    
    if exportEnabled ~= 0  
        frame = getframe(fig);
        writeVideo(v,frame);
    end
end

if exportEnabled ~= 0 
    v.close()
end




end

