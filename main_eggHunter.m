close all;
clear;
clc;

camList = webcamlist;
disp('ca list:');
disp(camList);

if any(strcmp(camList, 'e2eSoft iVCam'))
    cam = webcam('e2eSoft iVCam');
else
    cam = webcam(length(camList));
end

img = snapshot(cam);
[height, width, ~] = size(img);

% === 创建窗口和坐标轴 ===
fig = figure('Position', [100, 100, width, height], 'Name','Egg Hunter');
ax = axes('Parent', fig);
axis(ax, 'image');  % 保持像素比例
hold(ax, 'on');

hImg = image(ax, img);
ax.XColor = 'none';
ax.YColor = 'none';
ax.TickLength = [0 0];
box(ax, 'off');

% 初始化网格框和中心点句柄数组
hRects = [];
hCenters = [];
hTitle = title(ax, 'Detecting the egg carton...');
sTitle = subtitle(ax, ' ');

% log
% fig_img = figure('Name','log');
% ax_img = axes('Parent', fig_img);
% axis(ax_img, 'image');
% hold(ax_img, 'on');
% 
% bw_empty = false(size(img,1), size(img,2));
% log_img = imshow(bw_empty, 'Parent', ax_img);
log_img = [];

robot = robot();

robot.base = [width/2,0];
robot.scale = 8.5;

bot_plot = robot.create_plot(ax);

q1 = deg2rad(90);
q2 = deg2rad(90);

draw_5link(robot, bot_plot, q1, q2);
draw_workspace(ax, robot, false, false);

task = EggTask([0, 0], [0, 0], robot);
eggCells = [];
eggs = [];

keyframeInterval = 0;
try
    while ishandle(fig)
        img = snapshot(cam);
        set(hImg, 'CData', img);

        % 任务开始
        % 检测轨道（是否固定位置？）
        % 显示框选轨道内的鸡蛋

        % 绘制五连杆机械臂动画：末端从初始位置移动到轨道里的鸡蛋上方。同时发送角度指令给 arduino 控制机械臂旋转
        % 修改 title 正在抓取
        % 等待5秒，代表已抓取鸡蛋
        % 绘制五连杆机械臂末端移动到空的鸡蛋盒格子上方
        % 修改 title 正在安置
        % 等待空盒子的状态改变（这里需要手动拿一下黄色纸片到指定位置），执行下一次任务
        
        isKeyFrame = false;
        if keyframeInterval == 20
            keyframeInterval = 0;
            isKeyFrame = true;
        end
        keyframeInterval = keyframeInterval + 1;

        if isKeyFrame
            % 检测鸡蛋盒子的网格
            [eggCells, trayRect] = trayCellDetection(img, log_img, robot);

            % 检测鸡蛋盒子外面的鸡蛋
            [eggs, eggValid] = findEggsNotInROI(img, trayRect, robot);
            if eggValid
                egg = eggs(1);
            else
                egg = Egg();
            end
        end

        if task.step == 0 && isKeyFrame && eggValid
            for i = 1:length(eggCells)
                cell = eggCells(i);
                if cell.valid && ~cell.hasEgg()
                    to = cell.center;
                    task = EggTask(egg.center, to, robot);
                    task.step = 1;
                    break;
                end
            end
        elseif task.step == 1
            if isKeyFrame
                task.beginCheck(egg);
            end
            if task.step == 1
                countdownTime = task.countdownTime();
                if countdownTime <= 0
                    task.begin(q1, q2);
                end
            end
        elseif task.step == 2
            [q1, q2] = task.move(bot_plot);
            pause(0.2);
        elseif task.step == 3 && isKeyFrame
            for i = 1:length(eggCells)
                cell = eggCells(i);
                if cell.valid && cell.hasEgg()
                    to = cell.center;
                    distance = norm(to - task.to);
                    ok = distance < cell.egg.radius / 2;
                    task.resultCheck(ok);
                    if ok
                        break;
                    end
                end
            end
        end
        
        [hRects, hCenters] = updateUI(ax, hTitle, sTitle, hRects, hCenters,...
            eggCells, eggs, task);

        % pause(0.1);
    end
catch ME
    disp(['Error: ', ME.message]);
    errorReport = getReport(ME, 'extended', 'hyperlinks', 'off');
    fprintf('Full error report:\n%s\n', errorReport);
end

% === 释放摄像头 ===
clear cam;
imaqreset;
disp('camera has freed');

%% Update UI %%
function [hRects, hCenters] = updateUI(ax, hTitle, sTitle, hRects, hCenters, ...
    eggCells, eggs, task)

if length(eggCells) < 1
    set(hTitle, 'String', 'Egg carton detection in progress');
    subtitle(ax, ' ');
end

cellCount = length(eggCells);
eggCount = length(eggs);
occupied = sum(arrayfun(@(x) ~isempty(x.egg), eggCells));
rectSize = occupied + length(eggs);
centerSize = cellCount + eggCount;

currentRectSize = length(hRects);
currentCenterSize = length(hCenters);

if currentRectSize < rectSize
    numToAdd = rectSize - currentRectSize;
    hRects = [hRects; gobjects(numToAdd, 1)];
    for i = currentRectSize + 1:rectSize
        hRects(i) = rectangle('Position', [0, 0, 1, 1], 'Curvature', [1, 1], 'EdgeColor', 'g', 'LineWidth', 1, 'Parent', ax, 'Visible', 'off');
    end
else
    for i = rectSize + 1:currentRectSize
        set(hRects(i), 'Visible', 'off');
    end
end

if currentCenterSize < centerSize
    numToAdd = centerSize - currentCenterSize;
    hCenters = [hCenters; gobjects(numToAdd, 1)];
    for i = currentCenterSize + 1:centerSize
        hCenters(i) = plot(ax, 0, 0, 'r+', 'MarkerSize', 6, 'LineWidth', 1, 'Visible', 'off');
    end
else
    for i = centerSize + 1:currentCenterSize
        set(hCenters(i), 'Visible', 'off');
    end
end

validCount = 0;
rectCount = 0;
for i = 1:centerSize
    if i > cellCount
        egg = eggs(i - cellCount);
        center = egg.center;
        if egg.valid
            centerColor = 'g';
        else
            centerColor = 'r';
        end
        circlePos = [center(1)-egg.radius, center(2)-egg.radius, egg.radius*2, egg.radius*2];

        rectCount = rectCount + 1;
        set(hRects(rectCount), 'Position', circlePos, 'Visible', 'on');
    else
        cell = eggCells(i);
        center = cell.center;
        valid = cell.valid;

        if cell.hasEgg()
            egg = cell.egg;
            centerColor = 'g';
            circlePos = [egg.center(1)-egg.radius, egg.center(2)-egg.radius, egg.radius*2, egg.radius*2];

            rectCount = rectCount + 1;
            set(hRects(rectCount), 'Position', circlePos, 'Visible', 'on');
        elseif valid
            centerColor = 'y';
        else
            validCount = validCount + 1;
            centerColor = 'r';
        end
    end
    set(hCenters(i), 'XData', center(1), 'YData', center(2), 'Color', centerColor, 'Visible', 'on');
end


% 更新标题
set(hTitle, 'String', sprintf('%d compartments found, with %d eggs loaded and %d compartment outside the workspace.', cellCount, occupied, validCount));

if task.step == 1
    subString = sprintf('Egg detected for grasping within the workspace. Grasping sequence in %d seconds.', task.countdownTime());
elseif task.step == 2
    subString = sprintf('Robot is executing the egg grasp sequence.');
elseif task.step == 3
    subString = sprintf('Grasp completed. Result Checking...');
elseif ~isempty(eggs) && ~eggs(1).valid
    subString = sprintf('Egg detected for grasping, but it is outside the workspace.');
else
    subString = ' ';
end
set(sTitle, 'String', subString);

end

%% Egg Carton Detection %%
function [eggCells, trayRect] = trayCellDetection(img, log_img, robot)

gray_img = rgb2gray(img);
bw = imbinarize(gray_img);

if ~isempty(log_img)
    set(log_img, 'CData', bw);
end

% 寻找所有连通区域
stats_all = regionprops(bw, 'BoundingBox', 'Area');
if isempty(stats_all)
    eggCells = [];
    trayRect = [];
    return;
end
% 找到面积最大的区域（白纸）
areas = [stats_all.Area];
[~, maxIdx] = max(areas);
% 获取白纸区域的边界框
paperBoundingBox = stats_all(maxIdx).BoundingBox;
% 在白纸区域内创建掩码
[rows, cols] = size(gray_img);
paperMask = false(rows, cols);
% 计算白纸区域的像素范围
x1 = max(1, floor(paperBoundingBox(1)));
y1 = max(1, floor(paperBoundingBox(2)));
x2 = min(cols, x1 + floor(paperBoundingBox(3)) - 1);
y2 = min(rows, y1 + floor(paperBoundingBox(4)) - 1);
paperMask(y1:y2, x1:x2) = true;
% 只在白纸区域内处理图像
paperGray = gray_img;
paperGray(~paperMask) = 0;
if ~isempty(log_img)
    set(log_img, 'CData', paperGray);
end
% 在白纸区域内二值化
paperBw = imbinarize(paperGray, 'adaptive', 'ForegroundPolarity','bright', 'Sensitivity', 0.8);
% 基于形态学操作检测网格线（只在白纸区域内
se_horizontal = strel('line', 30, 0); % 水平结构元素
se_vertical = strel('line', 30, 90); % 垂直结构元素
horizontal_lines = imopen(paperBw, se_horizontal);
vertical_lines = imopen(paperBw, se_vertical);
grid_lines = horizontal_lines | vertical_lines;
grid_lines = ~imclearborder(~grid_lines);
if ~isempty(log_img)
    set(log_img, 'CData', grid_lines);
end

% 寻找网格单元（只在白纸区域内）
stats_grid = regionprops(grid_lines, 'BoundingBox', 'Area', 'Centroid', 'Extent');

% 筛选网格单元
minCellArea = 2000;
maxCellArea = 8000;
N = length(stats_grid);

eggCells(N, 1) = EggCell();

x1_list = zeros(1, N);
y1_list = zeros(1, N);
x2_list = zeros(1, N);
y2_list = zeros(1, N);

k = 0;
for i = 1:N
    s = stats_grid(i);

    if s.Area > minCellArea && s.Area < maxCellArea && s.Extent > 0.5

        k = k + 1;

        centroid = s.Centroid;
        bb = s.BoundingBox;

        [~, ~, valid] = InvKin_5link(centroid, robot);
        
        eggCells(k).center = centroid;
        eggCells(k).valid = valid;

        if valid
            eggCells(k).egg = detectEggInROI(img, bb, robot);
        end

        x1_list(k) = bb(1);
        y1_list(k) = bb(2);
        x2_list(k) = bb(1) + bb(3);
        y2_list(k) = bb(2) + bb(4);
    end
end

eggCells     = eggCells(1:k);
x1_list      = x1_list(1:k);
y1_list      = y1_list(1:k);
x2_list      = x2_list(1:k);
y2_list      = y2_list(1:k);

if ~isempty(eggCells)
    centers = reshape([eggCells.center], 2, [])';
    validFlags = [eggCells.valid]';
    
    [~, idx] = sortrows([-double(validFlags), -centers(:, 2), -centers(:, 1)]);
    eggCells = eggCells(idx);
end

if isempty(x1_list)
    trayRect = [x1, y1, x2 - x1 + 1, y2 - y1 + 1];
else
    trayRect = [min(x1_list), min(y1_list), ...
                max(x2_list) - min(x1_list), ...
                max(y2_list) - min(y1_list)];
end
end


%% Find Egg %%
function [eggs, result] = findEggsNotInROI(img, rectPos, robot)

imgH = size(img,1);
imgW = size(img,2);

x = rectPos(1);
y = rectPos(2);
w = rectPos(3);
h = rectPos(4);

px = floor(x);
py = floor(y);
pw = floor(w);
ph = floor(h);

eggCells = {};  
cellIndex = 1;

% 四个白纸外区域：上、下、左、右
outerROIs = [
    1,       1,         imgW,         py-1;            % 上
    1,       py+ph,     imgW,         imgH-(py+ph);    % 下
    1,       py,        px-1,         ph;              % 左
    px+pw,   py,        imgW-(px+pw), ph               % 右
];

for k = 1:4
    roi = outerROIs(k, :);
    if roi(3) <= 0 || roi(4) <= 0
        continue;
    end

    detected = detectEggsInROI(img, roi, robot);
    if ~isempty(detected)
        n = numel(detected);
        eggCells(cellIndex : cellIndex+n-1) = num2cell(detected);
        cellIndex = cellIndex + n;
    end
end

if isempty(eggCells)
    eggs = Egg();
    result = false;
    return;
end

eggs = [eggCells{:}];

result = any([eggs.valid]);

validArr = [eggs.valid];
centers = reshape([eggs.center], 2, []).';
[~, idx] = sortrows([~validArr.', -centers(:,2), -centers(:,1)]);
eggs = eggs(idx);

end


%% Yello Detect %%
function egg = detectEggInROI(img, rectPos, robot)
    eggs = detectEggsInROI(img, rectPos, robot);
    if isempty(eggs)
        egg = Egg.empty;
        return;
    end
    egg = eggs(1);
end

function eggs = detectEggsInROI(img, rectPos, robot)

x1 = max(1, round(rectPos(1)));
y1 = max(1, round(rectPos(2)));
x2 = min(size(img, 2), round(rectPos(1) + rectPos(3)));
y2 = min(size(img, 1), round(rectPos(2) + rectPos(4)));

if x1 >= x2 || y1 >= y2
    return;
end

roi = img(y1:y2, x1:x2, :);

hsv_img = rgb2hsv(roi);

h = hsv_img(:, :, 1);
s = hsv_img(:, :, 2); 
v = hsv_img(:, :, 3);

% 定义黄色的HSV范围
% 黄色在HSV中的色调范围大约为 0.08-0.18 (对应15°-35°)
lower_h = 0.08;  upper_h = 0.18;
lower_s = 0.50;  upper_s = 1.00; % 饱和度阈值
lower_v = 0.50;  upper_v = 1.00; % 明度阈值

% 创建黄色掩码
yellow_mask = (h >= lower_h) & (h <= upper_h) & ...
    (s >= lower_s) & (s <= upper_s) & ...
    (v >= lower_v) & (v <= upper_v);

% 形态学操作
se = strel('disk',2);
img_diff = imclose(yellow_mask, se);

[centers, radii, metric] = imfindcircles(img_diff, [10 150]);

validIdx = find(metric > 0.3);

if isempty(validIdx)
    eggs = Egg.empty;
    return;
end

n = numel(validIdx);
eggs = Egg.empty(n,0);

for j = 1:n
    e = Egg();
    e.center = centers(validIdx(j), :) + [x1, y1];
    e.radius = radii(validIdx(j));
    [~, ~, e.valid] = InvKin_5link(e.center, robot);
    eggs(j) = e;
end
end
