%% Initialization
clc
close all
clear all
t=0;
SimLength=30;
TimeStep=.01;
RobotHistoryPlotStep=20;
m=zeros(1,SimLength/TimeStep*RobotHistoryPlotStep+1);
XRecord=nan*ones(1,SimLength/TimeStep+1);
YRecord=nan*ones(1,SimLength/TimeStep+1);
%% Recording results video
dlg_string = sprintf('Do you want to record a video of results?\n\nThe name of the file will be the current date and time and it will be saved in the same directory.');
choice=questdlg(dlg_string,'Video Recording','Yes','No','Cancel','No');
if(strcmp(choice,'Cancel')||isempty(choice)) 
    break
end
recordBool=strcmp(choice,'Yes');
if recordBool
    
    c=fix(clock);
    cstr=strcat(mat2str(c),'.avi');
    SimulationVideo = VideoWriter(cstr);
    open(SimulationVideo);
    axis tight
    set(gca,'nextplot','replacechildren');
    set(gcf,'Renderer','zbuffer');
end
%% Setup test components
Base=SimBase();
ImportMap(Base,'workspace.bmp');
agent=RobotStates();
%% Simulation
hold on
while t<SimLength&&abs((agent.X-agent.TargetX)^2+(agent.Y-agent.TargetY)^2)>100
    if(mod(int32(t/TimeStep),RobotHistoryPlotStep)==0)
        ShowField(agent);
    end
    Show(agent);
    MoveRobot(agent,Base,TimeStep);
    drawnow
    t=t+TimeStep;
    if recordBool
        frame = getframe;
        writeVideo(SimulationVideo,frame);
    end
    XRecord(round(t/TimeStep+1))=[agent.X];
    YRecord(round(t/TimeStep+1))=[agent.Y];
    clc
    display(t);
end
%% Finalize video
if recordBool
    close(SimulationVideo);
end