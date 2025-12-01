function draw_workspace(ax, robot, bottom, find_tray)

q1_range = linspace(0, pi, 50);  % 0-180
q2_range = linspace(0, pi, 50);  % 0-180

workspace_points = [];

for q1 = q1_range
    for q2 = q2_range
        joints = ForwKin_5link(q1, q2, robot);
        if length(joints) == 5
            workspace_points = [workspace_points; joints(5, :)];
        end
    end
end

workspace_objs = findobj(ax, 'Tag', 'workspace');
if ~isempty(workspace_objs)
    delete(workspace_objs);
end


if ~isempty(workspace_points)
    sc = scatter(ax, workspace_points(:,1), workspace_points(:,2), 10, ...
        [0.7 0.8 1.0], 'filled', 'MarkerEdgeAlpha', 0.3, ...
        'MarkerFaceAlpha', 0.4, 'Tag', 'workspace');
    if bottom
        uistack(sc, 'bottom');
    end

    [p, found] = find_rectangle_position(workspace_points, robot);
    if found && find_tray
        plot_max_rectangle(ax, p);
    end
end

if size(workspace_points, 1) > 2
    try
        k = convhull(workspace_points(:,1), workspace_points(:,2));
        p = plot(ax, workspace_points(k,1), workspace_points(k,2), 'c-', ...
            'LineWidth', 1.5, 'Tag', 'workspace');
        if bottom
            uistack(p, 'bottom');
        end
    catch
        p = plot(ax, workspace_points(:,1), workspace_points(:,2), 'c-', ...
            'LineWidth', 1.5, 'Tag', 'workspace');
        if bottom
            uistack(p, 'bottom');
        end
    end
end
end

function [rect_position, found] = find_rectangle_position(workspace_points, robot)
if isempty(workspace_points)
    rect_position = [];
    return;
end

pos_x = robot.base(1);

found = false;

x_min = min(workspace_points(:,1));
x_max = max(workspace_points(:,1));
y_min = min(workspace_points(:,2));
y_max = max(workspace_points(:,2));

x_range1 = linspace(pos_x, x_max, floor(x_max-pos_x));
x_range2 = linspace(pos_x, x_min, floor(pos_x-x_min));
x_range = [x_range1, x_range2];
[~, sort_idx] = sort(abs(x_range- pos_x));
x_range = x_range(sort_idx);

y_range = linspace(y_min , y_max , floor(y_max-y_min));

rect_position = [];
for x = x_range
    for y = y_range
        pass = true;
        points = tray_key_points([x, y]);
        for i = 1:size(points, 1)
            [~, ~, valid] = InvKin_5link(points(i, :), robot);
            if ~valid
                pass = false;
                break;
            end
        end

        if pass == true
            found = true;
            rect_position=[x, y];
            return;
        end
    end
end
end


function plot_max_rectangle(ax, p)

x_center = p(1); y_center = p(2);
width = 30; height = 25;

x_corners = [x_center - width/2, x_center + width/2, x_center + width/2, x_center - width/2, x_center - width/2];
y_corners = [y_center - height/2, y_center - height/2, y_center + height/2, y_center + height/2, y_center - height/2];

plot(ax, x_corners, y_corners, 'r-', 'LineWidth', 3, 'Tag', 'workspace');
fill(ax, x_corners, y_corners, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'Tag', 'workspace');

points = tray_key_points(p);
for i = 1:size(points, 1)
    point = points(i, :);
    plot(ax, point(1), point(2), 'r+', 'MarkerSize', 6, 'LineWidth', 2, 'Tag', 'workspace');
end

text(ax, x_center, y_center - height / 2 - 2, sprintf('%.0fx%.0f', width, height), ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'red', ...
    'Tag', 'workspace');
end