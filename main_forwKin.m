function create_ui(ax, height, robot, bot_plot)

x = 700;
y = height - 240;

q1 = 20;
q2 = 20;

q1_text = uicontrol('Style', 'text', 'Position', [x, y, 120, 20], ...
    'String', 'Q1:', 'FontSize', 10);
q1_slider = uicontrol('Style', 'slider', 'Position', [x + 120, y, 120, 20], ...
    'Min', 0, 'Max', 180, 'Value', q1);

y = y - 40;
q2_text = uicontrol('Style', 'text', 'Position', [x, y, 120, 20], ...
    'String', 'Q2:', 'FontSize', 10);
q2_slider = uicontrol('Style', 'slider', 'Position', [x+120, y, 120, 20], ...
    'Min', 0, 'Max', 180, 'Value', q2);

y = y - 40;
uicontrol('Style', 'text', 'Position', [x, y, 120, 20], ...
    'String', 'Base X:', 'FontSize', 10);
base_x_slider = uicontrol('Style', 'slider', 'Position', [x+120, y, 120, 20], ...
    'Min', -50, 'Max', 50, 'Value', 0);

y = y - 40;
uicontrol('Style', 'text', 'Position', [x, y, 120, 20], ...
    'String', 'Base Y:', 'FontSize', 10);
base_y_slider = uicontrol('Style', 'slider', 'Position', [x+120, y, 120, 20], ...
    'Min', -50, 'Max', 50, 'Value', 0);

y = y - 40;
uicontrol('Style', 'text', 'Position', [x, y, 120, 20], ...
    'String', 'scale:', 'FontSize', 10);
scale_slider = uicontrol('Style', 'slider', 'Position', [x+120, y, 120, 20], ...
    'Min', 0.5, 'Max', 2, 'Value', 1);

set(base_x_slider, 'Callback', @(src,evt) update_plot());
set(base_y_slider, 'Callback', @(src,evt) update_plot());
set(q1_slider, 'Callback', @(src,evt) update_plot());
set(q2_slider, 'Callback', @(src,evt) update_plot());
set(scale_slider, 'Callback', @(src,evt) update_plot());
    function update_plot()
        pos_x = get(base_x_slider, 'Value');
        pos_y = get(base_y_slider, 'Value');
        q1_deg = get(q1_slider, 'Value');
        q2_deg = get(q2_slider, 'Value');
        scale = get(scale_slider, 'Value');

        set(q1_text, 'String', sprintf('Q1:%.2f', q1_deg));
        set(q2_text, 'String', sprintf('Q2:%.2f', q2_deg));

        q1 = deg2rad(q1_deg);
        q2 = deg2rad(q2_deg);
        
        robot.base = [pos_x, pos_y];
        robot.scale = scale;

        draw_workspace(ax, robot, true, true);
        draw_5link(robot, bot_plot, q1, q2);
    end
update_plot();
end


close all;
clear;
clc;

robot = robot();

width = 1000;
height = 600;

figure('Position', [100, 100, width, height], 'Name', '5-link simulation');

main_ax = subplot('Position', [0.05, 0.1, 0.65, 0.8]);
axis equal;
grid on;
hold on;
xlabel('X');
ylabel('Y');
title('5-link simulation');

axis_limit = 60;
xlim([-axis_limit, axis_limit]);
ylim([-10, axis_limit]);

legend_ax = subplot('Position', [0.75, 0.7, 0.2, 0.2]);
axis off;
title('configuration');

bot_plot = robot.create_plot(main_ax);

link_params = {
    sprintf('L0 = %d (black)', robot.L0) ...
    sprintf('L1 = %d (red)', robot.L1), ...
    sprintf('L2 = %d (blue)', robot.L2), ...
    sprintf('L3 = %d (green)', robot.L3), ...
    sprintf('L4 = %d (magenta)', robot.L4), ...
    };

text(legend_ax, 0.1, 0.9, link_params, 'FontSize', 12, 'VerticalAlignment', 'top', ...
    'FontName', 'SimHei');

create_ui(main_ax, height, robot, bot_plot);