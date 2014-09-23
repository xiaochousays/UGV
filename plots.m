close all



Base=SimBase();
ImportMap(Base,'workspace.bmp');

hold on
plot(XGapFinding,YGapFinding,XPotentialAndGapFinding,YPotentialAndGapFinding...
    ,XPotentialField,YPotentialField,XPotentialAndGapFindingLessRepulsion,...
    YPotentialAndGapFindingLessRepulsion,XPotentialAndGapFindingLessRepulsionDenseLidar,...
    YPotentialAndGapFindingLessRepulsionDenseLidar)



for i=1:50:900
    plot(XGapFinding(i),YGapFinding(i),'o',XPotentialAndGapFinding(i),YPotentialAndGapFinding(i)...
    ,'o',XPotentialField(i),YPotentialField(i),'o',XPotentialAndGapFindingLessRepulsion(i)...
    ,YPotentialAndGapFindingLessRepulsion(i),'o',XPotentialAndGapFindingLessRepulsionDenseLidar(i),...
    YPotentialAndGapFindingLessRepulsionDenseLidar(i),'o')
end
legend('Gap Finding','Potential Field + Gap Finding','Potential Field','Potential Field And Gap Finding with less repulsion force','Potential And Gap Finding Less Repulsion Dense Lidar')


close all



Base=SimBase();
ImportMap(Base,'workspace.bmp');

hold on
plot(XGapFinding,YGapFinding,XPotentialAndGapFinding,YPotentialAndGapFinding...
    ,XPotentialField,YPotentialField,XPotentialAndGapFindingLessRepulsion,...
    YPotentialAndGapFindingLessRepulsion)



for i=1:50:900
    plot(XGapFinding(i),YGapFinding(i),'o',XPotentialAndGapFinding(i),YPotentialAndGapFinding(i)...
    ,'o',XPotentialField(i),YPotentialField(i),'o',XPotentialAndGapFindingLessRepulsion(i)...
    ,YPotentialAndGapFindingLessRepulsion(i),'o')
end
legend('Gap Finding','Potential Field + Gap Finding','Potential Field','Potential Field And Gap Finding with less repulsion force','Potential And Gap Finding Less Repulsion Dense Lidar')

