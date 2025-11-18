function draw_workspace(ax, robot)
% 绘制五连杆机械臂的工作空间
% 通过扫描所有可能的q1和q2角度组合

% 角度范围：0-180度
q1_range = linspace(0, pi, 50);  % 0-180度
q2_range = linspace(0, pi, 50);  % 0-180度

% 存储所有有效的末端位置
workspace_points = [];

% 扫描所有角度组合
for q1 = q1_range
    for q2 = q2_range
        joints = ForwKin_5link(q1, q2, robot);
        if length(joints) == 5
            workspace_points = [workspace_points; joints(5, :)];
        end
    end
end

% 清除之前的工作空间绘制
workspace_objs = findobj(ax, 'Tag', 'workspace');
if ~isempty(workspace_objs)
    delete(workspace_objs);
end

% 绘制工作空间点云
if ~isempty(workspace_points)
    sc = scatter(ax, workspace_points(:,1), workspace_points(:,2), 10, ...
        [0.7 0.8 1.0], 'filled', 'MarkerEdgeAlpha', 0.3, ...
        'MarkerFaceAlpha', 0.4, 'Tag', 'workspace');
    uistack(sc, 'bottom');
    % 计算并绘制最大内接矩形
    [p, found] = find_rectangle_position(workspace_points, robot);
    if found
        plot_max_rectangle(ax, p);
    end
end

% 绘制工作空间边界（凸包）
if size(workspace_points, 1) > 2
    try
        k = convhull(workspace_points(:,1), workspace_points(:,2));
        p = plot(ax, workspace_points(k,1), workspace_points(k,2), 'c-', ...
            'LineWidth', 1.5, 'Tag', 'workspace');
        uistack(p, 'bottom');
    catch
        % 如果凸包计算失败，直接连接边界点
        p = plot(ax, workspace_points(:,1), workspace_points(:,2), 'c-', ...
            'LineWidth', 1.5, 'Tag', 'workspace');
        uistack(p, 'bottom');
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

% 获取工作空间边界
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

% 矩形四个角点
x_corners = [x_center - width/2, x_center + width/2, x_center + width/2, x_center - width/2, x_center - width/2];
y_corners = [y_center - height/2, y_center - height/2, y_center + height/2, y_center + height/2, y_center - height/2];

% 绘制矩形
plot(ax, x_corners, y_corners, 'r-', 'LineWidth', 3, 'Tag', 'workspace');
fill(ax, x_corners, y_corners, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'Tag', 'workspace');

points = tray_key_points(p);
for i = 1:size(points, 1)
    point = points(i, :);
    plot(ax, point(1), point(2), 'r+', 'MarkerSize', 6, 'LineWidth', 2, 'Tag', 'workspace');
end

% 标注矩形尺寸
text(ax, x_center, y_center - height / 2 - 2, sprintf('%.0f×%.0f', width, height), ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'red', ...
    'Tag', 'workspace');
end