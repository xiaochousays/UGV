classdef Lidar < handle
    
    properties
        Type;
        DistanceRange=250;
        AngularRange=260;
        ScanStepSize=1;
        LidarPlot; 
    end
    
    
    methods
        
        function obj=Lidar()
        end
            
        function [LidarReadingAngles,LidarReadingDistances]=LidarData(Robot,Base)
            LidarPositionX=Robot.X+cos(Robot.Theta*pi/180)*Robot.LidarOffCenter;
            LidarPositionY=Robot.Y+sin(Robot.Theta*pi/180)*Robot.LidarOffCenter;
            Map=Base.Matrix;
            MapSize=size(Map);
            LineStep=1;

            if ishandle(Robot.LidarPlot)
                delete(Robot.LidarPlot)
            end
            LidarReadingAngles=(-Robot.AngularRange/2+Robot.ScanStepSize/2:Robot.ScanStepSize:Robot.AngularRange/2-Robot.ScanStepSize/2)';
            LidarReadingDistances=zeros(Robot.AngularRange/Robot.ScanStepSize,1);

            for i=1:Robot.AngularRange/Robot.ScanStepSize
                m=(LidarReadingAngles(i)+Robot.Theta)*pi/180;       
                TravelStep=1;
                exceed=0;
                
                while exceed==0
                    LightTravel=TravelStep*LineStep;
                    LightedPix(TravelStep,:)=[int16(LidarPositionX+LightTravel*cos(m)),int16(LidarPositionY+LightTravel*sin(m))];
                    
                    if LightedPix(TravelStep,1)>0&&LightedPix(TravelStep,1)<MapSize(2)&&LightedPix(TravelStep,2)>0&&LightedPix(TravelStep,2)<MapSize(1) %if inside map
                        
                        if Map(LightedPix(TravelStep,2),LightedPix(TravelStep,1))==1  %if hit a wall
                            LidarReadingDistances(i)=LightTravel;
                            exceed=1;
                        end
                        TravelStep=TravelStep+1;
                        
                        if LightTravel>Robot.DistanceRange 
                            LidarReadingDistances(i)=LightTravel-LineStep;
                            exceed=1;
                        end
                        
                    else
                        LidarReadingDistances(i)=LightTravel;
                        exceed=1;
                    end
                end
                Robot.LidarPlot(i)=line([LidarPositionX,LidarPositionX+LidarReadingDistances(i)*cos((LidarReadingAngles(i)+Robot.Theta)*pi/180)],[LidarPositionY,LidarPositionY+LidarReadingDistances(i)*sin((LidarReadingAngles(i)+Robot.Theta)*pi/180)],'Color','y','LineWidth',.1,'LineStyle',':');
            end
        end
        
        function PlotLidar(Sensor)
        end
        
    end
    
    
end