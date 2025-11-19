function joints = ForwKin_5link(q1, q2, robot)

scale = robot.scale;

% Link lengths
L0 = robot.L0 * scale;
L1 = robot.L1 * scale;
L2 = robot.L2 * scale;
L3 = robot.L3 * scale;
L4 = robot.L4 * scale;

% Base position
pos_x = robot.base(1);
pos_y = robot.base(2);

% Fixed base points
base_left  = [pos_x - L0/2, pos_y];
base_right = [pos_x + L0/2, pos_y];

% Active joint positions
joint1_pos = base_left  + L1 * [cos(q1), sin(q1)];
joint2_pos = base_right + L3 * [cos(q2), sin(q2)];

% Distance between active joints
d = norm(joint2_pos - joint1_pos);

% Reachability check
if d > (L2 + L4) || d < abs(L2 - L4)
    joints = [];
    return;
end

% Circle–circle intersection geometry
a = (L2^2 - L4^2 + d^2) / (2*d);
h = sqrt(L2^2 - a^2);

% Midpoint between the two intersection points
mid_point = joint1_pos + (a/d) * (joint2_pos - joint1_pos);

% Perpendicular vector (unit)
perp_vector = [ -(joint2_pos(2) - joint1_pos(2)), ...
                  joint2_pos(1) - joint1_pos(1) ] / d;

% Two possible end-effector positions
p1 = mid_point + h * perp_vector;
p2 = mid_point - h * perp_vector;

% Elbow-up selection using cross product
% Vector from left active joint → right active joint
v = joint2_pos - joint1_pos;

% Candidate vectors from midpoint → p1/p2
w1 = p1 - mid_point;

% 2D cross product z component
cross1 = v(1)*w1(2) - v(2)*w1(1);

% Elbow-up is traditionally defined as cross > 0
if cross1 > 0
    end_effector_pos = p1;
else
    end_effector_pos = p2;
end

% Output joints in consistent order
joints = [
    base_left;
    base_right;
    joint1_pos;
    joint2_pos;
    end_effector_pos
];
end
