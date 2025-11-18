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

% === 创建窗口和坐标轴 ===
fig = figure('Name','Egg Hunter');
ax = axes('Parent', fig);
axis(ax, 'image');  % 保持像素比例
hold(ax, 'on');
hImg = imshow(img, 'Parent', ax);

% 初始化矩形和机械臂连杆的绘制对象
hRect = plot(ax, nan, nan, 'r-', 'LineWidth', 2);   % 红色矩形
hArm  = plot(ax, nan, nan, 'b-o', ...
    'LineWidth',2, ...
    'MarkerEdgeColor','black', ...
    'MarkerFaceColor','white', ...
    'MarkerSize',4);

% 初始化网格框和中心点句柄数组
hCircles = [];
hCenters = [];
hTitle = title(ax, '初始化中...');

% log
fig_img = figure('Name','log');
ax_img = axes('Parent', fig_img);
axis(ax_img, 'image');  % 保持像素比例
hold(ax_img, 'on');

bw_empty = false(size(img,1), size(img,2));
log_img = imshow(bw_empty, 'Parent', ax_img);

try
    while ishandle(fig)
        % 拍摄一帧并更新图像
        img = snapshot(cam);
        set(hImg, 'CData', img);

        % 检测鸡蛋盒子的网格
        [validCells, validCentroids] = trayCellDetection(img, log_img);
        % 更新鸡蛋盒子和鸡蛋显示
        [hCircles, hCenters] = updateEggDisplayInTray(img, ax, hTitle, hCircles, hCenters, validCells, validCentroids);

        % 任务开始
        % 检测轨道（是否固定位置？）
        % 显示框选轨道内的鸡蛋

        % 绘制五连杆机械臂动画：末端从初始位置移动到轨道里的鸡蛋上方。同时发送角度指令给 arduino 控制机械臂旋转
        % 修改 title 正在抓取
        % 等待5秒，代表已抓取鸡蛋
        % 绘制五连杆机械臂末端移动到空的鸡蛋盒格子上方
        % 修改 title 正在安置
        % 等待空盒子的状态改变（这里需要手动拿一下黄色纸片到指定位置），执行下一次任务

        pause(0.01);
    end
catch ME
    disp(['Error: ', ME.message]);
end

% === 释放摄像头 ===
clear cam;
imaqreset;
disp('摄像头已释放');

function [validCells, validCentroids] = trayCellDetection(img, log_img)
% 图像预处理和鸡蛋检测
gray_img = rgb2gray(img);
bw = imbinarize(gray_img);
set(log_img, 'CData', bw);

% 寻找所有连通区域
stats_all = regionprops(bw, 'BoundingBox', 'Area');

if isempty(stats_all)
    validCells = [];
    validCentroids = [];
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

set(log_img, 'CData', paperGray);

% 在白纸区域内二值化
paperBw = imbinarize(paperGray, 'adaptive', 'Sensitivity', 0.7);

% 基于形态学操作检测网格线（只在白纸区域内）
se_horizontal = strel('line', 30, 0);   % 水平结构元素
se_vertical = strel('line', 30, 90);    % 垂直结构元素

horizontal_lines = imopen(paperBw, se_horizontal);
vertical_lines = imopen(paperBw, se_vertical);
grid_lines = horizontal_lines | vertical_lines;

grid_lines = ~imclearborder(~grid_lines);
set(log_img, 'CData', grid_lines);

% 寻找网格单元（只在白纸区域内）
stats_grid = regionprops(grid_lines, 'BoundingBox', 'Area', 'Centroid', 'Extent');

% 筛选网格单元
minCellArea = 5000;
maxCellArea = 8000;
validCells = [];
validCentroids = [];

for i = 1:length(stats_grid)
    if stats_grid(i).Area > minCellArea && stats_grid(i).Area < maxCellArea && stats_grid(i).Extent > 0.5 % 确保是矩形
        validCells = [validCells; stats_grid(i).BoundingBox];
        validCentroids = [validCentroids; stats_grid(i).Centroid];
    end
end
end

function [hCircles, hCenters] = updateEggDisplayInTray(img, ax, hTitle, hCircles, hCenters, validCells, validCentroids)
% 更新鸡蛋检测显示

% 如果找到足够多的网格单元
if size(validCells, 1) < 1
    set(hTitle, 'String', '未检测到网格单元');
    return;
end

% 确保有足够的图形对象
numCells = size(validCells, 1);
currentNumHandles = length(hCircles);

% 如果句柄数量不够，扩大缓存
if currentNumHandles < numCells
    % 计算需要新增的数量
    numToAdd = numCells - currentNumHandles;

    % 扩展句柄数组
    hCircles = [hCircles; gobjects(numToAdd, 1)];
    hCenters = [hCenters; gobjects(numToAdd, 1)];

    % 创建新增的图形对象
    for i = currentNumHandles + 1:numCells
        % 创建矩形框
        hCircles(i) = rectangle('Position', [0, 0, 1, 1], 'Curvature', [1, 1], 'EdgeColor', 'g', 'LineWidth', 2, 'Parent', ax, 'Visible', 'off');
        % 创建中心点
        hCenters(i) = plot(ax, 0, 0, 'r+', 'MarkerSize', 8, 'LineWidth', 2, 'Visible', 'off');
    end

    % 如果句柄数量太多，删除多余的
elseif currentNumHandles > numCells
    % 删除多余的图形对象
    for i = numCells + 1:currentNumHandles
        if isvalid(hCircles(i))
            delete(hCircles(i));
        end
        if isvalid(hCenters(i))
            delete(hCenters(i));
        end
    end

    % 截断句柄数组
    hCircles = hCircles(1:numCells);
    hCenters = hCenters(1:numCells);
end

% 更新标题
set(hTitle, 'String', sprintf('检测到 %d 个网格单元', numCells));

% 统一更新所有图形对象的位置和颜色
for i = 1:numCells
    center = validCentroids(i, :);
    cellBoundingBox = validCells(i, :);

    % 检测当前矩形框内是否有黄色圆形
    [hasYellowCircle, circleCenter, circleRadius] = detectYellowCircleInROI(img, cellBoundingBox);

    % 根据检测结果设置颜色
    if hasYellowCircle
        centerColor = 'g'; % 绿色
        circlePos = [circleCenter(1)-circleRadius, circleCenter(2)-circleRadius, circleRadius*2, circleRadius*2];
        set(hCircles(i), 'Position', circlePos, 'Visible', 'on');
    else
        centerColor = 'r'; % 红色
        set(hCircles(i), 'Visible', 'off');
    end
    % 更新中心点位置和颜色
    set(hCenters(i), 'XData', center(1), 'YData', center(2), 'Color', centerColor, 'Visible', 'on');
end
end

% 黄色圆形检测函数
function [hasYellow, circleCenter, circleRadius] = detectYellowCircleInROI(img, rectPos)
hasYellow = false;
circleCenter = [0, 0];
circleRadius = 0;

% 提取矩形区域
x1 = max(1, round(rectPos(1)));
y1 = max(1, round(rectPos(2)));
x2 = min(size(img, 2), round(rectPos(1) + rectPos(3)));
y2 = min(size(img, 1), round(rectPos(2) + rectPos(4)));

% 确保区域有效
if x1 >= x2 || y1 >= y2
    return;
end

roi = img(y1:y2, x1:x2, :);

% 转换为HSV颜色空间进行黄色检测
hsv_img = rgb2hsv(roi);

% 提取H, S, V通道
h = hsv_img(:, :, 1); % 色调 (范围: 0-1)
s = hsv_img(:, :, 2); % 饱和度 (范围: 0-1)
v = hsv_img(:, :, 3); % 明度 (范围: 0-1)

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

[centers,radii,metric] = imfindcircles(img_diff,[10 150]);

% 如果有检测到圆形且置信度足够高，则认为有黄色圆形
hasYellow = ~isempty(centers) && any(metric > 0.3);
if hasYellow
    [~, bestIdx] = max(metric);
    bestCenter = centers(bestIdx, :);
    bestRadius = radii(bestIdx);

    circleCenter = [x1 + bestCenter(1), y1 + bestCenter(2)];
    circleRadius = bestRadius;
end
end