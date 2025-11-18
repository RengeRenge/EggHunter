function main_grid_detection()
    % 主函数：实时网格检测
    
    % 初始化摄像头
    cam = initializeCamera();
    if isempty(cam)
        error('无法初始化摄像头');
    end
    
    % 初始化图形窗口
    fig = figure;
    ax = axes('Parent', fig);
    
    % 初始化图形对象句柄
    [hImage, hRectangles, hCenters, hPaperRect, hTitle] = initializeGraphics(ax);
    
    try
        while ishandle(fig)
            % 拍摄一帧并更新图像
            img = snapshot(cam);
            
            % 处理当前帧并检测网格
            [validCells, validCentroids, paperBoundingBox] = processFrame(img);
            
            % 更新图形显示
            updateGraphics(ax, hImage, hRectangles, hCenters, hPaperRect, hTitle, ...
                          img, validCells, validCentroids, paperBoundingBox);
            
            drawnow;
        end
        
    catch ME
        disp(['Error: ', ME.message]);
        if exist('cam', 'var')
            clear cam;
        end
    end
    
    % 清理摄像头
    if exist('cam', 'var')
        clear cam;
    end
end

function cam = initializeCamera()
    % 初始化摄像头
    
    camList = webcamlist;
    disp('摄像头列表:');
    disp(camList);
    
    if isempty(camList)
        error('未检测到可用的摄像头');
    end
    
    try
        % 优先选择 e2eSoft iVCam
        if any(strcmp(camList, 'e2eSoft iVCam'))
            cam = webcam('e2eSoft iVCam');
            disp('使用 e2eSoft iVCam 摄像头');
        else
            % 使用最后一个摄像头
            cam = webcam(length(camList));
            disp(['使用默认摄像头: ', camList{length(camList)}]);
        end
        
        % 测试摄像头
        testImg = snapshot(cam);
        disp(['摄像头分辨率: ', num2str(size(testImg, 2)), 'x', num2str(size(testImg, 1))]);
        
    catch ME
        disp(['摄像头初始化失败: ', ME.message]);
        cam = [];
    end
end

function [hImage, hRectangles, hCenters, hPaperRect, hTitle] = initializeGraphics(ax)
    % 初始化所有图形对象
    
    % 初始图像（使用默认尺寸）
    hImage = imshow(zeros(480, 640, 3, 'uint8'), 'Parent', ax);
    hold(ax, 'on');
    
    % 初始化图形对象句柄数组
    hRectangles = [];    % 网格矩形框句柄
    hCenters = [];       % 中心点句柄
    hPaperRect = [];     % 白纸区域矩形句柄
    hTitle = title(ax, '初始化中...');
end

function [validCells, validCentroids, paperBoundingBox] = processFrame(img)
    % 处理单帧图像，检测白纸区域和网格
    
    % 转换为灰度图像
    if size(img, 3) == 3
        gray_img = rgb2gray(img);
    else
        gray_img = img;
    end
    
    % 检测白纸区域
    [paperBoundingBox, paperMask] = detectPaperRegion(gray_img);
    
    % 在白纸区域内检测网格
    [validCells, validCentroids] = detectGridInRegion(gray_img, paperMask);
end

function [paperBoundingBox, paperMask] = detectPaperRegion(gray_img)
    % 检测白纸区域（最大连通区域）
    
    % 二值化处理
    bw = imbinarize(gray_img);
    
    % 寻找所有连通区域
    stats_all = regionprops(bw, 'BoundingBox', 'Area');
    
    if isempty(stats_all)
        paperBoundingBox = [];
        paperMask = false(size(gray_img));
        return;
    end
    
    % 找到面积最大的区域（白纸）
    areas = [stats_all.Area];
    [maxArea, maxIdx] = max(areas);
    paperBoundingBox = stats_all(maxIdx).BoundingBox;
    
    % 创建白纸区域掩码
    [rows, cols] = size(gray_img);
    paperMask = false(rows, cols);
    
    % 计算白纸区域的像素范围
    x1 = max(1, floor(paperBoundingBox(1)));
    y1 = max(1, floor(paperBoundingBox(2)));
    x2 = min(cols, x1 + floor(paperBoundingBox(3)) - 1);
    y2 = min(rows, y1 + floor(paperBoundingBox(4)) - 1);
    
    if x1 <= x2 && y1 <= y2
        paperMask(y1:y2, x1:x2) = true;
    end
end

function [validCells, validCentroids] = detectGridInRegion(gray_img, paperMask)
    % 在白纸区域内检测网格
    
    [rows, cols] = size(gray_img);
    
    % 如果没有有效的白纸区域，返回空结果
    if ~any(paperMask(:))
        validCells = [];
        validCentroids = [];
        return;
    end
    
    % 在白纸区域内处理图像
    paperGray = gray_img;
    paperGray(~paperMask) = 0;
    
    % 在白纸区域内二值化
    paperBw = imbinarize(paperGray);
    
    % 基于形态学操作检测网格线
    se_horizontal = strel('line', 20, 0);   % 水平结构元素
    se_vertical = strel('line', 20, 90);    % 垂直结构元素
    
    horizontal_lines = imopen(paperBw, se_horizontal);
    vertical_lines = imopen(paperBw, se_vertical);
    grid_lines = horizontal_lines | vertical_lines;
    
    % 寻找网格单元（只在白纸区域内）
    stats_grid = regionprops(~grid_lines & paperMask, 'BoundingBox', 'Area', 'Centroid');
    
    % 筛选网格单元
    minCellArea = 300;
    maxCellArea = 5000;
    validCells = [];
    validCentroids = [];
    
    for i = 1:length(stats_grid)
        if stats_grid(i).Area > minCellArea && stats_grid(i).Area < maxCellArea
            bb = stats_grid(i).BoundingBox;
            aspect_ratio = bb(3) / bb(4);
            if aspect_ratio > 0.6 && aspect_ratio < 1.8
                validCells = [validCells; bb];
                validCentroids = [validCentroids; stats_grid(i).Centroid];
            end
        end
    end
end

function updateGraphics(ax, hImage, hRectangles, hCenters, hPaperRect, hTitle, ...
                       img, validCells, validCentroids, paperBoundingBox)
    % 更新图形显示
    
    % 更新图像数据
    set(hImage, 'CData', img);
    
    % 更新白纸区域显示
    updatePaperRegionDisplay(ax, hPaperRect, paperBoundingBox);
    
    % 更新网格显示
    updateGridDisplay(ax, hRectangles, hCenters, hTitle, validCells, validCentroids);
end

function updatePaperRegionDisplay(ax, hPaperRect, paperBoundingBox)
    % 更新白纸区域显示
    
    if ~isempty(paperBoundingBox)
        if isempty(hPaperRect)
            % 创建白纸区域矩形
            hPaperRect = rectangle('Position', paperBoundingBox, ...
                                  'EdgeColor', 'g', 'LineWidth', 2, 'LineStyle', '--', ...
                                  'Parent', ax);
        else
            % 更新白纸区域矩形位置
            set(hPaperRect, 'Position', paperBoundingBox, 'Visible', 'on');
        end
    elseif ~isempty(hPaperRect)
        set(hPaperRect, 'Visible', 'off');
    end
end

function updateGridDisplay(ax, hRectangles, hCenters, hTitle, validCells, validCentroids)
    % 更新网格显示
    
    % 如果找到足够多的网格单元
    if size(validCells, 1) >= 20
        % 估算平均格子大小
        avgCellSize = mean(validCells(:, 3:4), 1);
        
        % 确保有足够的图形对象
        numCells = size(validCells, 1);
        
        % 如果句柄数组为空或数量不匹配，重新创建图形对象
        if isempty(hRectangles) || length(hRectangles) ~= numCells
            % 删除旧的图形对象
            if ~isempty(hRectangles)
                delete(hRectangles);
                delete(hCenters);
            end
            
            % 创建新的图形对象
            hRectangles = gobjects(numCells, 1);
            hCenters = gobjects(numCells, 1);
            
            for i = 1:numCells
                center = validCentroids(i, :);
                rectPos = [center(1)-avgCellSize(1)/2, center(2)-avgCellSize(2)/2, avgCellSize(1), avgCellSize(2)];
                
                % 创建矩形框
                hRectangles(i) = rectangle('Position', rectPos, ...
                                          'EdgeColor', 'r', 'LineWidth', 2, 'Parent', ax);
                
                % 创建中心点
                hCenters(i) = plot(ax, center(1), center(2), 'r+', ...
                                  'MarkerSize', 8, 'LineWidth', 2);
            end
        else
            % 更新现有的图形对象
            for i = 1:numCells
                center = validCentroids(i, :);
                rectPos = [center(1)-avgCellSize(1)/2, center(2)-avgCellSize(2)/2, avgCellSize(1), avgCellSize(2)];
                
                % 更新矩形框位置
                set(hRectangles(i), 'Position', rectPos);
                
                % 更新中心点位置
                set(hCenters(i), 'XData', center(1), 'YData', center(2));
            end
        end
        
        % 更新标题
        set(hTitle, 'String', sprintf('检测到 %d 个网格单元', numCells));
        
        % 显示所有对象
        set(hRectangles, 'Visible', 'on');
        set(hCenters, 'Visible', 'on');
        
    else
        % 没有检测到足够网格时隐藏图形对象
        if ~isempty(hRectangles)
            set(hRectangles, 'Visible', 'off');
            set(hCenters, 'Visible', 'off');
        end
        set(hTitle, 'String', '未检测到完整网格');
    end
end