close all;
clear;
clear class;
clc;

% scale iamge, enhance process speed.
procScale = 0.4;

camList = webcamlist;
disp('ca list:');
disp(camList);
camname = 'e2eSoft iVCam';

if any(strcmp(camList, camname))
    cam = webcam(camname);
else
    cam = webcam(length(camList));
end

img = snapshot(cam);
[height, width, ~] = size(img);

fig = figure('Position', [100, 100, width, height], 'Name','Egg Hunter');
ax = axes('Parent', fig);
axis(ax, 'image');
hold(ax, 'on');

hImg = image(ax, img);
ax.XColor = 'none';
ax.YColor = 'none';
ax.TickLength = [0 0];
box(ax, 'off');

hRects = [];
hCenters = [];
hTitle = title(ax, 'Detecting the egg carton...');
sTitle = subtitle(ax, ' ');

% log
fig_img = figure('Name','log');
ax_img = axes('Parent', fig_img);
axis(ax_img, 'image');
hold(ax_img, 'on');

bw_empty = false(size(img,1), size(img,2));
log_img = imshow(bw_empty, 'Parent', ax_img);
% log_img = [];

servo1 = ServoController('D9', 'COM15');
servo2 = ServoController('D8', 'COM15');

robot = robot();

robot.base = [width/2,0];
robot.scale = 8.5;

bot_plot = robot.create_plot(ax);

q1 = deg2rad(90);
q2 = deg2rad(90);
servo1.moveTo(q1);
servo2.moveTo(q2);
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

        isKeyFrame = false;
        if keyframeInterval == 20
            keyframeInterval = 0;
            isKeyFrame = true;
        end
        keyframeInterval = keyframeInterval + 1;

        if isKeyFrame
            [eggCells, trayRect] = trayCellDetection(img, log_img, robot);
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
            servo1.moveTo(q1);
            servo2.moveTo(q2);
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

% release camera
clear cam;
imaqreset;
disp('camera has released');

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
%% trayCellDetection %%
function [eggCells, trayRect] = trayCellDetection(img, log_img, robot, procScale)
if nargin < 4, procScale = 1.0; end

gray_small = imresize(rgb2gray(img), procScale);
if ~isempty(log_img), set(log_img, 'CData', imresize(gray_small, 1/procScale)); end
bw_small = imbinarize(gray_small);

stats_all = regionprops(bw_small, 'BoundingBox', 'Area');
if isempty(stats_all)
    eggCells = []; trayRect = []; return;
end

areas = [stats_all.Area];
[~, maxIdx] = max(areas);
paperBB_small = stats_all(maxIdx).BoundingBox;

[rows_s, cols_s] = size(gray_small);
paperMask_s = false(rows_s, cols_s);
x1 = max(1, floor(paperBB_small(1)));
y1 = max(1, floor(paperBB_small(2)));
x2 = min(cols_s, x1 + floor(paperBB_small(3)) - 1);
y2 = min(rows_s, y1 + floor(paperBB_small(4)) - 1);
paperMask_s(y1:y2, x1:x2) = true;

paperGray_s = gray_small;
paperGray_s(~paperMask_s) = 0;
if ~isempty(log_img), set(log_img, 'CData', imresize(paperGray_s, 1/procScale)); end

paperBw_s = imbinarize(paperGray_s, 'adaptive', 'Sensitivity', 0.9);

se_horizontal = strel('line', round(20*procScale), 0);
se_vertical = strel('line', round(20*procScale), 90);
horizontal_lines = imopen(paperBw_s, se_horizontal);
vertical_lines = imopen(paperBw_s, se_vertical);
grid_lines_s = horizontal_lines | vertical_lines;

grid_lines_s = ~imclearborder(~grid_lines_s);

% Make the line thicker %
grid_lines_s = imerode(grid_lines_s, strel('square', round(3*procScale)));
if ~isempty(log_img), set(log_img, 'CData', imresize(grid_lines_s, 1/procScale)); end

stats_grid = regionprops(grid_lines_s, 'BoundingBox', 'Area', 'Centroid', 'Extent');
minCellArea = max(20, round(2000 * procScale^2));
maxCellArea = round(8000 * procScale^2);
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
        centroid_orig = s.Centroid / procScale;
        bb_small = s.BoundingBox;
        bb_orig = [bb_small(1), bb_small(2), bb_small(3), bb_small(4)] ./ procScale;

        [~, ~, valid] = InvKin_5link(centroid_orig, robot);

        eggCells(k) = EggCell();
        eggCells(k).center = centroid_orig;
        eggCells(k).valid = valid;
        if valid
            eggCells(k).egg = detectEggInROI(img, bb_orig, robot, procScale);
        end

        x1_list(k) = bb_orig(1);
        y1_list(k) = bb_orig(2);
        x2_list(k) = bb_orig(1) + bb_orig(3);
        y2_list(k) = bb_orig(2) + bb_orig(4);
    end
end

eggCells = eggCells(1:k);
x1_list = x1_list(1:k);
y1_list = y1_list(1:k);
x2_list = x2_list(1:k);
y2_list = y2_list(1:k);

if isempty(eggCells)
    trayRect = [0,0,0,0];
    return;
end

centers = reshape([eggCells.center], 2, [])';
validFlags = [eggCells.valid]';
[~, idx] = sortrows([-double(validFlags), -centers(:,2), -centers(:,1)]);
eggCells = eggCells(idx);

trayRect = [min(x1_list), min(y1_list), max(x2_list) - min(x1_list), max(y2_list) - min(y1_list)];
end


%% findEggsNotInROI %%
function [eggs, result] = findEggsNotInROI(img, rectPos, robot, procScale)
if nargin < 4, procScale = 1.0; end
imgH = size(img,1); imgW = size(img,2);

x = rectPos(1); y = rectPos(2); w = rectPos(3); h = rectPos(4);
px = floor(x); py = floor(y); pw = floor(w); ph = floor(h);

eggCells = {};
cellIndex = 1;

outerROIs = [
    1,       1,         imgW,         py-1;
    1,       py+ph,     imgW,         imgH-(py+ph);
    1,       py,        px-1,         ph;
    px+pw,   py,        imgW-(px+pw), ph
    ];

for k = 1:4
    roi = outerROIs(k, :);
    if roi(3) <= 0 || roi(4) <= 0, continue; end
    detected = detectEggsInROI(img, roi, robot, procScale);
    if ~isempty(detected)
        n = numel(detected);
        eggCells(cellIndex:cellIndex + n - 1) = num2cell(detected);
        cellIndex = cellIndex + n;
    end
end

if isempty(eggCells)
    eggs = Egg(); result = false; return;
end

eggs = [eggCells{:}];
result = any([eggs.valid]);

validArr = [eggs.valid];
centers = reshape([eggs.center], 2, []).';
[~, idx] = sortrows([~validArr.', -centers(:,2), -centers(:,1)]);
eggs = eggs(idx);
end


%% detectEggInROI %%
function egg = detectEggInROI(img, rectPos, robot, procScale)
if nargin < 4, procScale = 1.0; end
eggs = detectEggsInROI(img, rectPos, robot, procScale);
if isempty(eggs), egg = Egg.empty; return; end
egg = eggs(1);
end

%% detectEggsInROI %%
function eggs = detectEggsInROI(img, rectPos, robot, procScale)
if nargin < 4, procScale = 1.0; end

x1 = max(1, round(rectPos(1)));
y1 = max(1, round(rectPos(2)));
x2 = min(size(img, 2), round(rectPos(1) + rectPos(3)));
y2 = min(size(img, 1), round(rectPos(2) + rectPos(4)));
if x1 >= x2 || y1 >= y2, eggs = Egg.empty; return; end

roi = img(y1:y2, x1:x2, :);

roi_small = imresize(roi, procScale);
hsv_img = rgb2hsv(roi_small);
h = hsv_img(:, :, 1); s = hsv_img(:, :, 2); v = hsv_img(:, :, 3);

lower_h = 0.08;  upper_h = 0.18;
lower_s = 0.50;  upper_s = 1.00;
lower_v = 0.50;  upper_v = 1.00;

yellow_mask_small = (h >= lower_h) & (h <= upper_h) & ...
    (s >= lower_s) & (s <= upper_s) & ...
    (v >= lower_v) & (v <= upper_v);

se = strel('disk', max(1, round(2*procScale)));
img_diff_small = imclose(yellow_mask_small, se);

minR = max(2, round(10 * procScale));
maxR = max(6, round(150 * procScale));
try
    [centers_s, radii_s, metric] = imfindcircles(img_diff_small, [minR maxR]);
catch
    centers_s = []; radii_s = []; metric = [];
end

validIdx = find(metric > 0.3);
if isempty(validIdx)
    eggs = Egg.empty;
    return;
end

n = numel(validIdx);
eggs = Egg.empty(n,0);
for j = 1:n
    idx = validIdx(j);
    c_s = centers_s(idx, :); r_s = radii_s(idx);

    center_orig = (c_s ./ procScale) + [x1-1, y1-1];
    radius_orig = r_s ./ procScale;

    e = Egg();
    e.center = center_orig;
    e.radius = radius_orig;
    [~, ~, e.valid] = InvKin_5link(e.center, robot);
    eggs(j) = e;
end
end