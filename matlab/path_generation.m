clc
clear
close all 

addpath('core_functions');

start = [2000, 0]
target = [2000, 5000]

% Different variants of obstacles may be considered for demonstration purposes
%obstacles = [1900 1600; 2300 1900; 2050 2000; 1800 2700; 2100 3500];
%obstacles = [2000 -2500];
obstacles = [1700 1600; 2300 1900; 2050 2000; 1800 2700; 3500 3500];


% Generate collision-free path
[path, vectors, vectors_len, path_len] = generatePath(start,target, obstacles);

%save('path_data');

% Add a collumn of zeros for the generatePath result
% We assume that the robot body orinenation is constant: only linear motion occurs  
vectors = [vectors zeros(size(vectors,1),1)];
vectors = transpose(vectors);

% Get wheel angular velocities (RAD) for the entire path
All_Vw = vectors2Vw(vectors);

% Plotting
figure
hold on
%Plotting Path
p = plot(path(:,1), path(:,2), 'LineWidth', 2);
p.Color = 'blue';
plot(obstacles(:,1), obstacles(:,2), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'black');
plot(start(1), start(2), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'blue');
plot(target(1), target(2), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'red');

title("Robot Path on the 2D map");
grid on
xlim([0 4000])
ylim([0 5200])

xlabel('X [mm]') 
ylabel('Y [mm]') 
axis equal

% Plot joint velocities (RAD) note that there are only two sets of joint velocites
% even if we have 4 wheels; this is true as long as no rotation happends
figure
plot(path_len(:,1), All_Vw(:,:), 'LineWidth', 2);

ylabel('Joint speed [Rad/s]') 
xlabel('Distance traveled [mm]') 
title("Joint speeds with respect to the linear distance traveled");
%figure
%plot(vectors(1,:), vectors(2,:), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'black');

%figure
%plot(path_len(:,1), All_Vw(1,:), 'LineWidth', 2);
%figure
%plot(path_len(:,1), All_Vw(2,:), 'LineWidth', 2);
%figure
%plot(path_len(:,1), All_Vw(3,:), 'LineWidth', 2);
%figure
%plot(path_len(:,1), All_Vw(4,:), 'LineWidth', 2);
