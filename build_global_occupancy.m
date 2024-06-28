% clear all; clc; close all;

% Build a large enough occupancy map - stored in a matrix or an
% occupancy_map object in navigation tool box for better display.

x_global_size = 20;
y_global_size = 20;
grid_size = 0.05;
occupancy_Map_global = zeros(ceil(x_global_size/grid_size), ceil(y_global_size/grid_size));

% Visualize the initial occupancy map
%imagesc(occupancy_Map_global)
%xlabel('X-axis');
%ylabel('Y-axis');

% Save the global occupancy map
save('global_occupancy_map.mat', 'occupancy_Map_global');