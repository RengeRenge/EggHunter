function joints = ForwKin_5link(q1, q2, robot)

scale = robot.scale;

% 连杆长度
L0 = robot.L0 * scale;
L1 = robot.L1 * scale;
L2 = robot.L2 * scale;
L3 = robot.L3 * scale;
L4 = robot.L4 * scale;
pos_x = robot.base(1);
pos_y = robot.base(2);

% 基座两个固定点
base_left = [pos_x - L0/2, pos_y];
base_right = [pos_x + L0/2, pos_y];

% 计算主动关节位置
joint1_pos = base_left + L1 * [cos(q1), sin(q1)];
joint2_pos = base_right + L3 * [cos(q2), sin(q2)];

% 计算两个从动杆的交点（末端执行器位置）
d = norm(joint2_pos - joint1_pos);

% 检查可达性
if d > (L2 + L4) || d < abs(L2 - L4)
    joints = [];
    return;
end

% 使用圆的交点计算方法
% 以joint1_pos为圆心，L2为半径的圆
% 以joint2_pos为圆心，L4为半径的圆

% 计算交点
a = (L2^2 - L4^2 + d^2) / (2 * d);
h = sqrt(L2^2 - a^2);

% 中点
mid_point = joint1_pos + (a / d) * (joint2_pos - joint1_pos);

% 两个可能的交点（上下配置）
% 垂直方向向量
perp_vector = [- (joint2_pos(2) - joint1_pos(2)), (joint2_pos(1) - joint1_pos(1))] / d;

% 两个可能的末端位置
end_effector_pos1 = mid_point + h * perp_vector;
end_effector_pos2 = mid_point - h * perp_vector;

end_effector_pos = end_effector_pos1;

% 通常选择使机械臂更"自然"的配置（比如末端在基座上方）
% 这里选择y坐标较大的点（假设基座在下方）
% if end_effector_pos1(2) > end_effector_pos2(2)
%     end_effector_pos = end_effector_pos1;
% else
%     end_effector_pos = end_effector_pos2;
% end

% 问题3：关节顺序可能需要调整以保持一致性
joints = [base_left;    % 1: 左侧基座
    base_right;   % 2: 右侧基座
    joint1_pos;   % 3: 左侧主动关节
    joint2_pos;   % 4: 右侧主动关节
    end_effector_pos]; % 5: 末端执行器
end