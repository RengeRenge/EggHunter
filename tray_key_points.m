function points = tray_key_points(center)
width = 30;
height = 25;
side = 5;

i = center(1);
j = center(2);

points = zeros(30, 2);
index = 1;
for offset_x = 0:5
    x = i - width/2 + side/2 + offset_x * side;
    for offset_y = 0:4
        y = j - height/2 + side/2 + offset_y * side;
        points(index, :) = [x, y];
        index = index + 1;
    end
end
end