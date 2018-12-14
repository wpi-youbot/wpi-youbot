clc
clear
close all 

addpath('core_functions');
addpath('3rdParty_functions');

start = [2000, 0]
target = [2000, 5000]

obstacles = [1800 2500];
%obstacles = [2000 -2500];
%obstacles = [1700 1600; 2300 1900; 2050 2000; 1800 2700; 3500 3500];

% Generate collision-free path
[path, vectors, vectors_len, path_len] = generatePath(start,target, obstacles);


centriFugal_data = [];

incr = 10; 
for it=(1+incr):(size(path,1)-incr)
    pts = [path(it-incr,:);
           path(it,:);
           path(it+incr,:)];

    res = CircleFitByPratt(pts)
    centriFugal_data = [centriFugal_data; res];
end

%size_of_radii = size(centriFugal_data)
%sum_non_inf = sum(isfinite(centriFugal_data))


% Plotting
figure
hold on
%Plotting Path
p = plot(path(:,1), path(:,2), 'LineWidth', 2);
p.Color = 'blue';
plot(obstacles(:,1), obstacles(:,2), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'black');
plot(start(1), start(2), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'blue');
plot(target(1), target(2), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'red');

% Plot centers of rotation
plot(centriFugal_data(:,1), centriFugal_data(:,2), 'o', 'MarkerSize', 1, 'MarkerFaceColor', 'blue');

grid on
xlim([0 4000])
ylim([0 5200])

xlabel('X [mm]') 
ylabel('Y [mm]') 
axis equal

xlim([0 4000])
ylim([0 5200])


% Plot radii smaller than 10000 mm
indices = find(abs(centriFugal_data)>10000);
centriFugal_data(indices) = NaN;

figure(2)
plot(path_len(1+incr:size(path_len)-incr), centriFugal_data(:,3), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'blue');
%plot(centriFugal_data(:,3), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'blue');
xlabel('Distance traveled [mm]') 
ylabel('R [mm]') 
axis equal

ylim([0 10200])
xlim([0 6000])

%size_path_len = size(path_len(1+incr:size(path_len)-incr))
%size_path_len = size(path_len)
%size_fugal = size(centriFugal_data(:,3))
%size_path = size(path)
%size_vectors_len = size(vectors_len)
