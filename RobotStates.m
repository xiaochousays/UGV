classdef RobotStates < handle & Lidar
    properties
        Name;
        X=150;
        Y=120;
        Theta=0;
        Length=100;
        Width=50;
        LidarOffCenter=40;
        TangentVelocity=0;
        AngularVelocity=0;
        MaxTangentVelocity=200;
        SafetyRadius;
        Rank=1;
        FieldNumber=5;
        TargetX=1600;
        TargetY=400;
        RobotFigures;
        FieldPlot=[];
        ArrowsPlot=[8,8,8];
        Mass=1;
        FieldDampDistMemory;
        EdgePlot;
        show_subgoals=1;
        show_field=1;
    end
    methods
        function obj=RobotStates()
            obj=obj@Lidar();
            obj.FieldDampDistMemory=zeros(obj.AngularRange/obj.ScanStepSize+1,1);
            obj.SafetyRadius=.6*sqrt(obj.Length^2+obj.Width^2);
            %if(nargin==1)
            %end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Show
        function Show(obj)
            D=20;
            if ishandle(obj.RobotFigures)
                delete(obj.RobotFigures);
            end
            [PatchX,PatchY]=RobotCorners(obj);
            obj.RobotFigures(1)=line([PatchX,PatchX(1)],[PatchY,PatchY(1)],'Color','w');       %shows robot
            XLidar=obj.X+cos(obj.Theta*pi/180)*obj.LidarOffCenter;
            YLidar=obj.Y+sin(obj.Theta*pi/180)*obj.LidarOffCenter;
            obj.RobotFigures(2)=rectangle('Position',[XLidar-D/2,YLidar-D/2,D,D],...    %shows lidar
                'Curvature',[1,1],...
                'LineWidth',1.5,'edgecolor','r');
            obj.RobotFigures(3)=line([XLidar,XLidar+D*cos(obj.Theta*pi/180)/2],[YLidar,YLidar+D*sin(obj.Theta*pi/180)/2],'Color','r','LineWidth',3);    %shows rest of lidar
            daspect([1,1,1]);
            obj.RobotFigures(4)=rectangle('Position',[obj.TargetX-10,obj.TargetY-10,20,20],...    %shows target
                'Curvature',[1,1],...
                'LineWidth',1.5,'edgecolor','c');
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Robot Corners
        function [Xs,Ys]=RobotCorners(obj)
            Xs=[...     %Robot corners
                obj.X+cos(obj.Theta*pi/180)*(obj.Length/2)+sin(obj.Theta*pi/180)*obj.Width/2,...
                obj.X+cos(obj.Theta*pi/180)*(obj.Length/2)-sin(obj.Theta*pi/180)*obj.Width/2,...
                obj.X-cos(obj.Theta*pi/180)*(obj.Length/2)-sin(obj.Theta*pi/180)*obj.Width/2,...
                obj.X-cos(obj.Theta*pi/180)*(obj.Length/2)+sin(obj.Theta*pi/180)*obj.Width/2];
            Ys=[...
                obj.Y+sin(obj.Theta*pi/180)*(obj.Length/2)-cos(obj.Theta*pi/180)*(obj.Width/2),...
                obj.Y+sin(obj.Theta*pi/180)*(obj.Length/2)+cos(obj.Theta*pi/180)*(obj.Width/2),...
                obj.Y-sin(obj.Theta*pi/180)*(obj.Length/2)+cos(obj.Theta*pi/180)*(obj.Width/2),...
                obj.Y-sin(obj.Theta*pi/180)*(obj.Length/2)-cos(obj.Theta*pi/180)*(obj.Width/2)];
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Show Field
        function ShowField(obj)
            if (obj.show_field==0)
                if (ishandle(obj.FieldPlot) )
                    delete(obj.FieldPlot)
                end
            end
            obj.SafetyRadius=.6*sqrt(obj.Length^2+obj.Width^2);
            k=3;
            C=2;
            Er=0;%abs(obj.TangentVelocity/(C*obj.MaxTangentVelocity));
            PlotTheta=(0:360)*pi/180;
            temp=1;
            %for e=0:Er/(obj.FieldNumber-1):Er
            e=0;
            PlotDist=(k*e*obj.SafetyRadius*obj.Rank./(1-e*cos(PlotTheta))+obj.SafetyRadius);
            obj.FieldPlot(temp)=plot(PlotDist.*cos(PlotTheta+obj.Theta*pi/180)+obj.X, PlotDist.*sin(PlotTheta+obj.Theta*pi/180)+obj.Y,'c');
            temp=temp+1;
            %end
            clear temp;
            daspect([1,1,1]);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Lidar Data
        function [LidarReadingAngles,LidarReadingDistances]=LidarData(obj,Base)
            [LidarReadingAngles,LidarReadingDistances]=LidarData@Lidar(obj,Base);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Translate to robot refrence
        function [DistancesVectorRobotRefrence,ThetasVectorRobotRefrence]=LidarToRobotCenter(DistancesVectorLidarReference,ThetasVectorLidarReference,obj)
            DistancesVectorRobotRefrenceX=DistancesVectorLidarReference.*cos(ThetasVectorLidarReference*pi/180)+obj.LidarOffCenter;
            DistancesVectorRobotRefrenceY=DistancesVectorLidarReference.*sin(ThetasVectorLidarReference*pi/180);
            DistancesVectorRobotRefrence=sqrt(DistancesVectorRobotRefrenceX.^2+DistancesVectorRobotRefrenceY.^2);
            ThetasVectorRobotRefrence=atan2(DistancesVectorRobotRefrenceY,DistancesVectorRobotRefrenceX)*180/pi;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Force of each reading
        function ForceMag=ForceArrayOfReading(DistancesVectorRobotRefrence,ThetasVectorRobotRefrence,obj,TimeStep)
            NumberOfReadings=length(DistancesVectorRobotRefrence);
            obj.SafetyRadius=.6*sqrt(obj.Length^2+obj.Width^2);
            DistancesVectorRobotRefrence=DistancesVectorRobotRefrence-obj.SafetyRadius;
            k=3;
            C=2;
            rho0=.1;
            ForceMax=0;
            ForceMag=zeros(NumberOfReadings,1);
            Er=abs(obj.TangentVelocity/(C*obj.MaxTangentVelocity));
            Dmax=k*Er*obj.SafetyRadius*obj.Rank./(1-Er*cos(ThetasVectorRobotRefrence));
            Dmin=rho0*Dmax;
            DampRatio=0;
            for i=1:NumberOfReadings
                if Dmax(i)<10
                    Dmax(i)=10;
                end
                if DistancesVectorRobotRefrence(i)>=Dmax(i)
                    ForceMag(i)=0;
                else
                    ForceMag(i)=(Dmax(i)-DistancesVectorRobotRefrence(i))*ForceMax/(Dmax(i)-Dmin(i))...
                        -DampRatio*(DistancesVectorRobotRefrence(i)-obj.FieldDampDistMemory(i))/TimeStep;
                end
            end
            obj.FieldDampDistMemory=DistancesVectorRobotRefrence;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Repulsion force calculation
        function [RepulsionForceXTotal,RepulsionForceYTotal]=RepulsionForce(ForceMag,ThetasVectorRobotRefrence,obj)
            RepulsionForceX=cos(ThetasVectorRobotRefrence*pi/180).*ForceMag;
            RepulsionForceY=sin(ThetasVectorRobotRefrence*pi/180).*ForceMag;
            RepulsionForceXTotal=sum(RepulsionForceX);
            RepulsionForceYTotal=sum(RepulsionForceY);
            RepulsionForceMagTotal=sqrt(RepulsionForceYTotal^2+RepulsionForceXTotal^2);
            RepulsionForceAngleTotal=atan2(RepulsionForceYTotal,RepulsionForceXTotal)+pi;
            %             if RepulsionForceMagTotal~=0
            %                 obj.ArrowsPlot(1)=arrow([obj.X obj.Y],[obj.X+RepulsionForceMagTotal*cos(RepulsionForceAngleTotal),...
            %                     obj.Y+RepulsionForceMagTotal*sin(RepulsionForceAngleTotal)], 'EdgeColor','r', 'FaceColor','r');
            %             end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Forces
        function [ForcesSummationMagnitude,ForcesSummationAngle]=...
                Forces(DistancesVectorLidarReference,ThetasVectorLidarReference,obj,TimeStep,AttractionForceAngle)
            % Repulsion
            [DistancesVectorRobotRefrence,ThetasVectorRobotRefrence]=LidarToRobotCenter(DistancesVectorLidarReference,ThetasVectorLidarReference,obj);
            ForceMag=ForceArrayOfReading(DistancesVectorRobotRefrence,ThetasVectorRobotRefrence,obj,TimeStep);
            for i=1:length(obj.ArrowsPlot)
                if ishandle(obj.ArrowsPlot(i))
                    delete(obj.ArrowsPlot(i));
                end
            end
            [RepulsionForceXTotal,RepulsionForceYTotal]=RepulsionForce(ForceMag,ThetasVectorRobotRefrence,obj);
            % attraction force
            Q=80;
            %             AttractionForceAngle=atan2(obj.TargetY-obj.Y,obj.TargetX-obj.X);
            %             obj.ArrowsPlot(2)=arrow([obj.X obj.Y],[obj.X+Q*cos(AttractionForceAngle), obj.Y+Q*sin(AttractionForceAngle)], 'EdgeColor','c', 'FaceColor','c');
            % Summation
            ForcesSummationX=-RepulsionForceXTotal+Q*cos(AttractionForceAngle);
            ForcesSummationY=-RepulsionForceYTotal+Q*sin(AttractionForceAngle);
            ForcesSummationMagnitude=sqrt(ForcesSummationX^2+ForcesSummationY^2);
            if ForcesSummationMagnitude<30
                ForcesSummationMagnitude=30;
            end
            ForcesSummationAngle=atan2(ForcesSummationY,ForcesSummationX);
            %             obj.ArrowsPlot(3)=arrow([obj.X obj.Y],[obj.X+ForcesSummationMagnitude*cos(ForcesSummationAngle),...
            %                 obj.Y+ForcesSummationMagnitude*sin(ForcesSummationAngle)],...
            %                 'EdgeColor','m', 'FaceColor','m');
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Potential Field
        function [ForcesSummationMagnitude,ForcesSummationAngle]=PotentialFieldController(obj,Base,TimeStep)
            [LidarReadingAngles,LidarReadingDistances]=LidarData(obj,Base);
            AttractionForceAngle=GapFinding(LidarReadingAngles,LidarReadingDistances,obj);
            [ForcesSummationMagnitude,ForcesSummationAngle]=...
                Forces(LidarReadingDistances,LidarReadingAngles,obj,TimeStep,AttractionForceAngle);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Obstacle Recognition
        function index=obstacleRecognition(obj,DistancesVectorLidarReference,ThetasVectorLidarReference)
            readings=length(DistancesVectorLidarReference);
            index=nan*ones(readings,1);
            noOfReturns=1;
            range=.95*obj.DistanceRange;
            for limitFinder=1:readings
                if DistancesVectorLidarReference(limitFinder)>range
                    DistancesVectorLidarReference(limitFinder)=nan;
                    ThetasVectorLidarReference(limitFinder)=nan;
                else
                    index(limitFinder)=noOfReturns;
                    noOfReturns=noOfReturns+1;
                end %if
            end %for
            DistancesVectorLidarReference(isnan(DistancesVectorLidarReference))=[];
            ThetasVectorLidarReference(isnan(ThetasVectorLidarReference))=[];
            if ~isempty(ThetasVectorLidarReference)
                [readingsX,readingsY]=pol2cart(ThetasVectorLidarReference*pi/180,DistancesVectorLidarReference);
                firstObstacleEnd=1;
                for obstacleEndIteration=1:int32(obj.AngularRange/obj.ScanStepSize)+1
                    for limitFinder=length(readingsX):-1:obstacleEndIteration
                        if ((readingsX(limitFinder)-readingsX(obstacleEndIteration))^2+(readingsY(limitFinder)-readingsY(obstacleEndIteration))^2)<=4*obj.SafetyRadius^2;
                            firstObstacleEnd=limitFinder;
                            break
                        end%if
                    end %for
                    if firstObstacleEnd==obstacleEndIteration
                        break % end found
                    end
                end
                numberOfObstacles=0;
                scanStart=firstObstacleEnd+1;
                obstacleElement=firstObstacleEnd+1;
                obstacleLimits=[nan,nan];
                while obstacleLimits(end)~=mod(firstObstacleEnd-1,length(DistancesVectorLidarReference))+1&&numberOfObstacles<10
                    numberOfObstacles=numberOfObstacles+1;
                    obstacleLimits(numberOfObstacles,1)=mod(scanStart-1,length(DistancesVectorLidarReference))+1;
                    obstacleLimits(numberOfObstacles,2)=scanStart;
                    % find end of the obstacle and begining and end of
                    % safe space
                    while obstacleLimits<=length(DistancesVectorLidarReference)+firstObstacleEnd+1
                        temp2=mod(obstacleElement-1,length(DistancesVectorLidarReference))+1;
                        for limitFinder=length(DistancesVectorLidarReference)+firstObstacleEnd:-1:obstacleElement
                            temp=mod(limitFinder-1,length(DistancesVectorLidarReference))+1;
                            if ((readingsX(temp)-readingsX(temp2))^2+(readingsY(temp)-readingsY(temp2))^2)<=4*obj.SafetyRadius^2;
                                obstacleLimits(numberOfObstacles,2)=temp;
                                break
                            end%if
                        end %for
                        obstacleElement=obstacleElement+1;
                        if obstacleLimits(numberOfObstacles,2)==temp2
                            scanStart=obstacleLimits(numberOfObstacles,2)+1;
                            break % end found
                        end
                        
                    end
                end
                [~,index] = ismember(obstacleLimits,index);
            else
                index=[];
            end
        end %obstacleRecognition
        %% LineTracking
        function [CommandTangentVelocity,CommandAngularVelocity]=LineTracking(Magnitude,Angle,obj)
            K1=.7;  %.7
            K2=.1; %.05
            K3=0; %1.5
            MagnitudeRatio=1;
            ErrorX=MagnitudeRatio*Magnitude*cos(Angle-obj.Theta*pi/180);
            ErrorY=MagnitudeRatio*Magnitude*sin(Angle-obj.Theta*pi/180);
            ErrorTheta=wrapToPi(Angle-obj.Theta*pi/180);
            FeedbackTangentVelocity=-K1*ErrorX;
            FeedbackAngularVelocity=-K2*sign(obj.TangentVelocity)*ErrorY-K3*ErrorTheta;
            CommandTangentVelocity=obj.TangentVelocity*cos(ErrorTheta)-FeedbackTangentVelocity;
            CommandAngularVelocity=-FeedbackAngularVelocity;
            VelocityRight=CommandTangentVelocity+CommandAngularVelocity*obj.Width/2;
            VelocityLeft=CommandTangentVelocity-CommandAngularVelocity*obj.Width/2;
            if abs(VelocityRight)>obj.MaxTangentVelocity
                VelocityRight=obj.MaxTangentVelocity*sign(VelocityRight);
            end
            if abs(VelocityLeft)>obj.MaxTangentVelocity
                VelocityLeft=obj.MaxTangentVelocity*sign(VelocityLeft);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% MoveRobot
        function obj=MoveRobot(obj,Base,TimeStep)
            [Magnitude,Angle]=PotentialFieldController(obj,Base,TimeStep);
            [CommandTangentVelocity,CommandAngularVelocity]=LineTracking(Magnitude,Angle,obj);
            [ForceLeft,ForceRight]=VelocityToForce(CommandTangentVelocity,CommandAngularVelocity,obj);
            [TangentAcceleration,AngularAcceleration]=ForceToActuation(ForceLeft,ForceRight,obj);
            obj.TangentVelocity=TangentAcceleration*TimeStep+obj.TangentVelocity;
            obj.AngularVelocity=AngularAcceleration*TimeStep+obj.AngularVelocity;
            obj.X=obj.X+obj.TangentVelocity*cos(obj.Theta*pi/180)*TimeStep;
            obj.Y=obj.Y+obj.TangentVelocity*sin(obj.Theta*pi/180)*TimeStep;
            obj.Theta=obj.Theta+obj.AngularVelocity*180*TimeStep/pi;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%
        function [ForceLeft,ForceRight]=VelocityToForce(CommandTangentVelocity,CommandAngularVelocity,obj)
            TangGain=30;
            AngGain=300;
            ForceRight=TangGain*((CommandTangentVelocity-obj.TangentVelocity)...
                +AngGain*(CommandAngularVelocity-obj.AngularVelocity)/obj.Width);
            ForceLeft=TangGain*((CommandTangentVelocity-obj.TangentVelocity)...
                -AngGain*(CommandAngularVelocity-obj.AngularVelocity)/obj.Width);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%
        function [TangentAcceleration,AngularAcceleration]=ForceToActuation(ForceLeft,ForceRight,obj)
            Damp=20;
            TangentAcceleration=(ForceRight+ForceLeft-abs(Damp*obj.TangentVelocity))/obj.Mass;
            AngularAcceleration=((ForceRight-ForceLeft)*obj.Width-abs(Damp*obj.AngularVelocity))/(obj.Mass*(obj.Width^2+obj.Length^2)/12);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%
        function [XX,YY]=robotReferenceToGlobal(obj,Angles,Distances)
            XX=obj.X+Distances.*cos((Angles+obj.Theta)*pi/180);
            YY=obj.Y+Distances.*sin((Angles+obj.Theta)*pi/180);
        end%function
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%
        function [subgoalsDistances,subgoalsTheta]=subgoals(lidarDistances,lidarThetas,obj)
            index=obstacleRecognition(obj,lidarDistances,lidarThetas);
            [robotDistances,robotThetas]=LidarToRobotCenter(lidarDistances,lidarThetas,obj);
            numberOfReadings=length(lidarDistances);
            numberOfObstacles=size(index,1);
            subgoalsTheta=nan*ones(numberOfObstacles,2);
            subgoalsDistances=nan*ones(numberOfObstacles,2);
            if ~isempty(index)
                if index(1,1)~=0
                    if index(1,2)<index(1,1)
                        index(1,2)=index(1,2)+numberOfReadings;
                    end
                    tem=mod((index(1,1):index(1,2))-1,numberOfReadings)+1;
                    temp=real([robotThetas(tem)-asind(obj.SafetyRadius./robotDistances(tem))]);
                    shift=min(temp);
                else
                    shift=0;
                end
                robotThetas=wrapTo360(robotThetas-shift);
                for i=1:numberOfObstacles
                    if index(i,2)<index(i,1)
                        index(i,2)=index(i,2)+numberOfReadings;
                    end
                    toCompare=index(i,1):1:index(i,2);
                    for j=toCompare
                        element=mod(j-1,numberOfReadings)+1;
                        temp1=real(robotThetas(element)-asind(obj.SafetyRadius/robotDistances(element)));
                        temp2=real(robotThetas(element)+asind(obj.SafetyRadius/robotDistances(element)));
                        if temp1<-.0001
                            temp1=temp1+360;
                            temp2=temp2+360;
                        end
                        temp3=subgoalsTheta(i,1);
                        temp4=subgoalsTheta(i,2);
                        [subgoalsTheta(i,1),ind]=min([temp3,temp1,temp2]);
                        if ind~=1
                            subgoalsDistances(i,1)=robotDistances(element);
                        end
                        [subgoalsTheta(i,2),ind]=max([temp4,temp1,temp2]);
                        if ind~=1
                            subgoalsDistances(i,2)=robotDistances(element);
                        end
                    end
                end
                subgoalsTheta=wrapTo360(subgoalsTheta+shift);
            else
                subgoalsTheta=[];
            end
        end%function
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function selectedSubgoal=selectSubgoal(obj,targetDistance,targetTheta,lidarDistances,lidarThetas)
            for i=1:length(obj.EdgePlot)
                if ishandle(obj.EdgePlot(i))
                    delete(obj.EdgePlot(i));
                end
            end
            [subgoalsDistances,subgoalsTheta]=subgoals(lidarDistances,lidarThetas,obj);
            targetTheta=wrapTo180(targetTheta);
            for i=1:size(subgoalsTheta,1)
                if subgoalsTheta(i,1)>subgoalsTheta(i,2)
                    subgoalsTheta(i,1)=subgoalsTheta(i,1)-360;
                end
            end
            subgoalsTheta2=wrapTo180(subgoalsTheta);
            difference=abs(targetTheta-subgoalsTheta2)+subgoalsDistances.*abs(wrapTo180(subgoalsTheta2))*.01;
            selectedSubgoal=[targetDistance,targetTheta];
            if ~isempty(subgoalsDistances)
                for i=1:size(subgoalsDistances,1)
                    diff=wrapTo180(subgoalsTheta(i,:)-selectedSubgoal(2));
                    if diff(1)>diff(2)
                        diff(1)=diff(1)-360;
                    end
                    if (sign(diff(1)*diff(2))==-1)&&selectedSubgoal(1)>sum(subgoalsDistances(i,:))/2
                        [~,Indx]=min([difference(i,1),difference(i,2)]);
                        selectedSubgoal=[subgoalsDistances(i,Indx),subgoalsTheta2(i,Indx)];
                    elseif (subgoalsTheta2(i,2)<subgoalsTheta2(i,1)||subgoalsTheta(i,2)>360)&&targetDistance>sum(lidarDistances(i,:))/2
                        if selectedSubgoal(2)<subgoalsTheta2(i,2)&&selectedSubgoal(2)>subgoalsTheta2(i,1)
                            [~,Indx]=max([difference(i,1),difference(i,2)]);
                            selectedSubgoal=[subgoalsDistances(i,Indx),subgoalsTheta2(i,Indx)];
                        end
                    end%if
                    
                end%for
            end
            
            if(obj.show_subgoals==1)
                for i=1:size(subgoalsDistances,1)
                    start=subgoalsTheta2(i,2);
                    stop=subgoalsTheta2(i,1);
                    angels=[start,stop];
                    start=subgoalsDistances(i,2);
                    stop=subgoalsDistances(i,1);
                    distances=[start,stop];
                    [XX,YY]=robotReferenceToGlobal(obj,angels,distances);
                    obj.EdgePlot(i)=plot(XX',YY','c:o','MarkerSize',10,'MarkerFaceColor','c','LineWidth',1);
                end%for
                
                
                [subgoalX,subgoalY]=robotReferenceToGlobal(obj,selectedSubgoal(2),selectedSubgoal(1));
                obj.EdgePlot(size(subgoalsDistances,1)+1)=rectangle('Position',[subgoalX-5,subgoalY-5,10,10],...    %shows subgoal
                    'Curvature',[1,1],'LineWidth',3,'edgecolor','m');
            end%if
        end%function
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%
        function Direction=GapFinding(LidarReadingAngles,LidarReadingDistances,obj)
            
            DistanceFromTarget=sqrt((obj.TargetX-obj.X)^2+(obj.TargetY-obj.Y)^2);
            TargetDirection=wrapTo180(atan2((obj.TargetY-obj.Y),(obj.TargetX-obj.X))*180/pi-obj.Theta);
            NumberOfReadings=length(LidarReadingAngles);  % from LIDAR
            SeeObstacle=zeros(NumberOfReadings+2,1);
            for i=1:NumberOfReadings  % does it see obstacle
                if (LidarReadingDistances(i)<obj.DistanceRange)&&(LidarReadingDistances(i)<DistanceFromTarget)
                    SeeObstacle(i+1)=1;
                else SeeObstacle(i+1)=0;
                end%if
            end%for
            targetTheta=TargetDirection;
            targetDistance=DistanceFromTarget;
            %             for i=1:4
            selectedSubgoal=selectSubgoal(obj,targetDistance,targetTheta,LidarReadingDistances,LidarReadingAngles);
            if targetDistance==selectedSubgoal(1)&&targetTheta==selectedSubgoal(2)
                
            else
                targetDistance=real(selectedSubgoal(1));
                targetTheta=real(selectedSubgoal(2));
            end%if
            %             end%for
            %Direction=atan2((obj.LidarOffCenter*sin(obj.Theta*pi/180)+LidarReadingDistances(ClosestNoObstacleReading)*sin((LidarReadingAngles(ClosestNoObstacleReading)+obj.Theta)*pi/180)),(obj.LidarOffCenter*cos(obj.Theta*pi/180)+LidarReadingDistances(ClosestNoObstacleReading)*cos((LidarReadingAngles(ClosestNoObstacleReading)+obj.Theta)*pi/180)));
            Direction=wrapTo180(targetTheta+obj.Theta)*pi/180;
        end     %GapFinding
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end %end methods
    
end