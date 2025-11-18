% function [q1, q2, valid] = InvKin_5link(pos_x, pos_y, end_effector, config, scale)
% % 逆运动学求解五连杆机械臂
% % 输入:
% %   pos_x, pos_y - 基座中心位置
% %   end_effector - 末端执行器目标位置 [x, y]
% %   config - 连杆长度配置 [L0, L1, L2, L3, L4]
% %   scale - 缩放因子
% % 输出:
% %   q1, q2 - 主动关节角度（弧度）
% %   valid - 是否找到有效解
% 
% % 连杆长度
% L0 = config(1) * scale;
% L1 = config(2) * scale;
% L2 = config(3) * scale;
% L3 = config(4) * scale;
% L4 = config(5) * scale;
% 
% % 基座两个固定点
% base_left = [pos_x - L0/2, pos_y];
% base_right = [pos_x + L0/2, pos_y];
% 
% % 末端执行器位置
% P = end_effector;
% 
% % 方法1: 几何法求解
% [q1_geo, q2_geo, valid_geo] = geometric_method(P, base_left, base_right, L0, L1, L2, L3, L4, scale);
% 
% if ~valid_geo
%     q1 = 0;
%     q2 = 0;
%     valid = false;
% else
%     q1 = q1_geo;
%     q2 = q2_geo;
%     valid = true;
% end
% end
% 
% function [q1, q2, valid] = geometric_method(P, base_left, base_right, L0, L1, L2, L3, L4, scale)
% % 几何法求解逆运动学
% 
% valid = false;
% q1 = 0;
% q2 = 0;
% 
% % 计算从末端到左侧基座的距离
% d_left = norm(P - base_left);
% % 计算从末端到右侧基座的距离
% d_right = norm(P - base_right);
% 
% % 检查可达性
% if d_left > (L1 + L2) || d_left < abs(L1 - L2) || ...
%         d_right > (L3 + L4) || d_right < abs(L3 - L4)
%     return;
% end
% 
% % 求解左侧关节角度
% % 使用余弦定理计算左侧三角形
% cos_alpha_left = (L1^2 + d_left^2 - L2^2) / (2 * L1 * d_left);
% if abs(cos_alpha_left) > 1
%     return;
% end
% 
% alpha_left = acos(cos_alpha_left);
% phi_left = atan2(P(2) - base_left(2), P(1) - base_left(1));
% 
% % 左侧有两个可能的解
% q1_1 = phi_left + alpha_left;
% q1_2 = phi_left - alpha_left;
% 
% % 求解右侧关节角度
% % 使用余弦定理计算右侧三角形
% cos_alpha_right = (L3^2 + d_right^2 - L4^2) / (2 * L3 * d_right);
% if abs(cos_alpha_right) > 1
%     return;
% end
% 
% alpha_right = acos(cos_alpha_right);
% phi_right = atan2(P(2) - base_right(2), P(1) - base_right(1));
% 
% % 右侧有两个可能的解
% q2_1 = phi_right + alpha_right;
% q2_2 = phi_right - alpha_right;
% 
% % 选择最合适的解组合（通常选择使机械臂更自然的配置）
% % 这里选择使两个主动杆都向上的配置
% solutions = [q1_1, q2_1; q1_1, q2_2; q1_2, q2_1; q1_2, q2_2];
% 
% % 验证每个解并选择有效的
% for i = 1:size(solutions, 1)
%     test_q1 = solutions(i, 1);
%     test_q2 = solutions(i, 2);
%     if test_q1 < 0 || test_q1 >180 || test_q2 < 0 || test_q2 >180
%         continue;
%     end
%     % 使用正向运动学验证
%     joints = ForwKin_5link((base_left(1)+base_right(1))/2, base_left(2), ...
%         [L0, L1, L2, L3, L4]./scale, test_q1, test_q2, scale);
%     if ~isempty(joints)
%         actual_pos = joints(5, :); % 末端执行器位置
%         error = norm(actual_pos - P);
%         if error < 1e-6  % 允许的误差范围
%             q1 = test_q1;
%             q2 = test_q2;
%             valid = true;
%             return;
%         end
%     end
% end
% end





function [q1, q2, valid] = InvKin_5link(end_effector, robot)

pos_x = robot.base(1);
pos_y = robot.base(2);
scale = robot.scale;

valid = false;
px = end_effector(1);
py = end_effector(2);

% --- 读取连杆 ---
L0 = robot.L0 * scale;
L1 = robot.L1 * scale;
L2 = robot.L2 * scale;
L3 = robot.L3 * scale;
L4 = robot.L4 * scale;

% --- 基座左右点 ---
base_left  = [pos_x - L0/2, pos_y];
base_right = [pos_x + L0/2, pos_y];

% --- 第一步：求 joint1 的位置 ---
% joint1、joint2 必须满足：从动杆 L2、L4 与末端 px,py 形成一个四边形
% 因此 joint1、joint2 在以 px 为中心的两个圆上

% joint1 在距离 px py 为 L2 的圆上
% joint2 在距离 px py 为 L4 的圆上

% --- 第二步：joint1 还必须满足 base_left -> joint1 长度 = L1
% 这意味着 joint1 是 base_left 和末端 px 交点的圆交点

[P1, ~, ok1] = circle_intersection(base_left, L1, [px py], L2);
if ~ok1
    q1=[]; q2=[];
    return;
end

% --- 第三步：joint2 同理 ---
[Q1, ~, ok2] = circle_intersection(base_right, L3, [px py], L4);
if ~ok2
    q1=[]; q2=[];
    return;
end

% 五杆有两个构型：肘上、肘下
% 选其中一个（默认肘上）：
joint1 = P1;
joint2 = Q1;

% --- 第四步：求 q1, q2 ---
q1 = atan2(joint1(2)-base_left(2),  joint1(1)-base_left(1));
q2 = atan2(joint2(2)-base_right(2), joint2(1)-base_right(1));

if q1 < 0 || q1 >180 || q2 < 0 || q2 >180
    return;
end
valid = true;

end



%% -------------------------- 圆交点辅助函数 --------------------------
function [p1, p2, ok] = circle_intersection(c1, r1, c2, r2)
d = norm(c2 - c1);

if d > r1 + r2 || d < abs(r1 - r2)
    p1=[]; p2=[]; ok=false;
    return;
end

a = (r1^2 - r2^2 + d^2) / (2*d);
h = sqrt(max(r1^2 - a^2, 0));

mid = c1 + a * (c2 - c1) / d;

offset = h * [-(c2(2)-c1(2))/d,  (c2(1)-c1(1))/d];

p1 = mid + offset;
p2 = mid - offset;
ok = true;
end
