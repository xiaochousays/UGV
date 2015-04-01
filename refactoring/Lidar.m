classdef Lidar < handle
    
    properties
        Type;
        DistanceRange=250;
        AngularRange=260;
        ScanStepSize=5;
        LidarPlot; 
    end
    
    
    methods
        
        function obj=Lidar()
        end
            
        function [LidarReadingAngles,LidarReadingDistances]=LidarData(obj,lidar_position_x,lidar_position_y,lidar_theta,Map)
            MapSize=size(Map);
            LineStep=1;

            if ishandle(obj.LidarPlot)
                delete(obj.LidarPlot)
            end
            LidarReadingAngles=(-obj.AngularRange/2+obj.ScanStepSize/2:obj.ScanStepSize:obj.AngularRange/2-obj.ScanStepSize/2)'*pi/180;
            LidarReadingDistances=zeros(obj.AngularRange/obj.ScanStepSize,1);

            for i=1:obj.AngularRange/obj.ScanStepSize
                m=(LidarReadingAngles(i)+lidar_theta);       
                TravelStep=1;
                exceed=0;
                
                while exceed==0
                    LightTravel=TravelStep*LineStep;
                    LightedPix(TravelStep,:)=[int16(lidar_position_x+LightTravel*cos(m)),int16(lidar_position_y+LightTravel*sin(m))];
                    
                    if LightedPix(TravelStep,1)>0&&LightedPix(TravelStep,1)<MapSize(2)&&LightedPix(TravelStep,2)>0&&LightedPix(TravelStep,2)<MapSize(1) %if inside map
                        
                        if Map(LightedPix(TravelStep,2),LightedPix(TravelStep,1))==1  %if hit a wall
                            LidarReadingDistances(i)=LightTravel;
                            exceed=1;
                        end
                        TravelStep=TravelStep+1;
                        
                        if LightTravel>obj.DistanceRange 
                            LidarReadingDistances(i)=LightTravel-LineStep;
                            exceed=1;
                        end
                        
                    else
                        LidarReadingDistances(i)=LightTravel;
                        exceed=1;
                    end
                end
            end
            obj.LidarPlot=plot(lidar_position_x+LidarReadingDistances.*cos(lidar_theta+LidarReadingAngles),lidar_position_y+LidarReadingDistances.*sin(lidar_theta+LidarReadingAngles),'g:');
        end
        
        function PlotLidar(Sensor)
        end
        
    end
    
    
end