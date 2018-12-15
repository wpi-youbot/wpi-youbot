%% This is the Path and Trajectory Generator code. This code is utilizing 
% All of the functions developed for generating path and trajectories
% The inputs of the code is Start Point, End Point, Obstacles location, 
% the number of points to be extracted from the path and the trajectories
% final time.

clear
clc
close all


%% Initialization
start = [2000, 0];                                                   %start point coordinates in mm
target = [2000, 5000];                                               %Target point coordinates in mm
obstacles = [1700 1600; 2300 1900; 2050 2000; 1800 2300; 3500 3500]; %Obstacles Coordinates
number_of_points=8                                                   %Number of Via Points to be extracted from the path
% The more number of points the more polynomials to be blended.
tf=20                                                                %Trajectories Final time.



%% Path Generation
[path, vectors, vectors_len, path_len] = generatePath(start,target, obstacles);  %Generating Path Function
path=path/1000;                                                                  %Path points in meter
path_size=size(path,1);                                                          %Size of the obtained path array

        
%% Path Segmentation based on the number of points provided
step=floor(path_size/number_of_points)         %step taking in the path array
maximum_limit=step*number_of_points            %Maximum limit using this step size

path_x=zeros((number_of_points+2),1)
path_y=zeros((number_of_points+2),1)

path_x(1)=path(1,1)
path_y(1)=path(1,2)

path_x(end)=path(end,1)
path_y(end)=path(end,2)

for i=step:step:maximum_limit
    path_x((i/step)+1)=path(i,1)
    path_y((i/step)+1)=path(i,2)
end

%% Plotting the Generated Path along with the extracted points
figure(1)
plot(path(:,1),path(:,2),'LineWidth',2.5)
hold on
plot((obstacles(:,1)/1000),(obstacles(:,2)/1000),'o','MarkerSize',10)
hold on
plot(path_x,path_y,'s','MarkerEdgeColor','black','MarkerSize',10)
grid on
title('Path Planned in X-Y Coordinates','FontSize',12)
xlabel('X Coordinate in m','FontSize',12,'FontWeight','bold','Color','r')
ylabel('Y Coordinate in m','FontSize',12,'FontWeight','bold','Color','r')
legend('Planned Path','Obstacles','Via Points')

%% Generating Trajectories for Both X,Y Coordinates
[a1] = trajectory_blend_quintic(path_x',tf,1);   %Coefficients of X coordinate
[a2] = trajectory_blend_quintic(path_y',tf,2);   %Coefficients of Y coordinate

%% Saving the Cofficients to CSV Files
csvwrite('X_Cofficients.csv',a1)
csvwrite('Y_Cofficients.csv',a2)
