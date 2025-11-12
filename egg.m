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
fig = figure('Name','Detection Demo');
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

% 计算机械臂参数
[height, width, ~] = size(img);
offsetX = width/2;
offsetY = height/2;
L1 = height/4;
L2 = width/4;

try
    while ishandle(fig)
        % 拍摄一帧并更新图像
        img = snapshot(cam);
        set(hImg, 'CData', img);

        % === HSV黄色检测替换原来的蓝色检测 ===
        % 将RGB图像转换为HSV颜色空间
        hsv_img = rgb2hsv(img);
        
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

        stats = regionprops(yellow_mask, 'BoundingBox', 'Centroid', 'Area', 'MajorAxisLength', 'MinorAxisLength');

if ~isempty(stats)
    % 找到最大的区域
    [maxArea, idx] = max([stats.Area]);
    
    % 可选：添加形状验证（长宽比接近1表示近似方形）
    aspect_ratio = stats(idx).MajorAxisLength / stats(idx).MinorAxisLength;
    
    if aspect_ratio > 0.7 && aspect_ratio < 1.3  % 近似方形的长宽比范围
        bbox = stats(idx).BoundingBox;
        center = stats(idx).Centroid;
        
        % 矩形坐标
        x0 = bbox(1);
        y0 = bbox(2);
        width = bbox(3);
        height = bbox(4);
        
        RectangleX = [x0, x0+width, x0+width, x0, x0];
        RectangleY = [y0, y0, y0+height, y0+height, y0];
        set(hRect, 'XData', RectangleX, 'YData', RectangleY);

        % 逆运动学计算（保持原有逻辑）
        centers_temp = center;
        centers_temp(1) = centers_temp(1) - offsetX;
        centers_temp(2) = centers_temp(2) - offsetY;

        theta = Invkin_2link(L1,L2,centers_temp,false);
        [pos1,pos2] = ForwKin_2link(L1,L2,theta);

        X = [offsetX,pos1(1)+offsetX, pos2(1)+offsetX];
        Y = [offsetY,pos1(2)+offsetY, pos2(2)+offsetY];
        set(hArm, 'XData', X, 'YData', Y);
    else
        % 不是方形，清空显示
        set(hRect, 'XData', nan, 'YData', nan);
        set(hArm, 'XData', nan, 'YData', nan);
    end
else
    set(hRect, 'XData', nan, 'YData', nan);
    set(hArm, 'XData', nan, 'YData', nan);
end

        pause(0.01);
    end
catch ME
    disp(['Error: ', ME.message]);
end

% === 释放摄像头 ===
clear cam;
imaqreset;
disp('摄像头已释放');