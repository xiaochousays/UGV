clc;clear all;close all;

lidar=Lidar()
safety_edge=40;

str='../workspace.bmp'
Matrix=flipud(imread(str));
Width=size(Matrix,1);
Length=size(Matrix,2);


BaseFigure = figure;
set(BaseFigure,'name','Figure of the workspace','numbertitle','off')
imshow(Matrix);
hold on
set(gca, 'Ydir', 'normal');
set(gca, 'XLim', [1 Length], 'YLim', [1 Width]);

robot_x=450;
robot_y=300;
robot_theta = 90*pi/180;
target_theta = 100*pi/180;

plot(robot_x,robot_y,'ro');
arrow_length=200;
quiver(robot_x,robot_y,arrow_length/4*cos(robot_theta),arrow_length/4*sin(robot_theta));
quiver(robot_x,robot_y,arrow_length*cos(target_theta),arrow_length*sin(target_theta),'r');

waitforbuttonpress()
[angles,distances]=lidar.LidarData(robot_x,robot_y,robot_theta,Matrix);

direction=1
for i=1:length(angles)
    h=plot(robot_x+distances(i).*cos(robot_theta+angles(i)),robot_y+distances(i).*sin(robot_theta+angles(i)),'bo');
    %calculate distance from line
    clc
    read_angle=angles(i)-(target_theta-robot_theta)
    dist=sin(read_angle)*distances(i)
    if abs(dist)<safety_edge
        critical_index=i
        new_target_theta_left = angles(i)+robot_theta-asin(safety_edge/distances(i))
        quiver(robot_x,robot_y,arrow_length*cos(new_target_theta_left),arrow_length*sin(new_target_theta_left),'r');
        new_target_theta_right = angles(i)+robot_theta+asin(safety_edge/distances(i))
        quiver(robot_x,robot_y,arrow_length*cos(new_target_theta_right),arrow_length*sin(new_target_theta_right),'r');
        break
    end
    % drawnow
	pause(.1)
    delete(h)
end

% while true
    
    
% end
