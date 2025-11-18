close all;
clear;
clc;

robot = robot();

width = 1600;
height = 1200;

figure('Position', [200, -10, width, height], 'Name', '5-link trajectory simulation group 2');

circle_ax = subplot(1,1,1);
axis equal;
grid on;
hold on;
xlabel('X');
ylabel('Y');
title('5-link trajectory simulation group 2');

xlim([-30, 30]);
ylim([-5, 50]);

t = 0:1:50;
x_traj = 1*cos(2*pi*t/50);
y_traj = 1*sin(2*pi*t/50) + 40;
traj = [x_traj; y_traj];

draw_trajectory(circle_ax, robot, traj);

x_traj = [linspace(1, 1, 10), linspace(1, -1, 20), linspace(-1, -1, 20), linspace(-1, 1, 20), linspace(1, 1, 10)];
y_traj = [linspace(40, 41, 10), linspace(41, 41, 20), linspace(41, 39, 20), linspace(39, 39, 20), linspace(39, 40, 10)];
traj = [x_traj; y_traj];
draw_trajectory(circle_ax, robot, traj);


