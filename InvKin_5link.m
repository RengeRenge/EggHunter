function [q1, q2, valid] = InvKin_5link(end_effector, robot)

pos_x = robot.base(1);
pos_y = robot.base(2);
scale = robot.scale;

valid = false;
px = end_effector(1);
py = end_effector(2);

% --- Link lengths ---
L0 = robot.L0 * scale;
L1 = robot.L1 * scale;
L2 = robot.L2 * scale;
L3 = robot.L3 * scale;
L4 = robot.L4 * scale;

% --- Base anchor points ---
base_left  = [pos_x - L0/2, pos_y];
base_right = [pos_x + L0/2, pos_y];

% --- Step 1: joint1 candidates ---
[P1, P2, ok1] = circle_intersection(base_left, L1, [px py], L2);
if ~ok1
    q1=[]; q2=[]; return;
end

% --- Step 2: joint2 candidates ---
[Q1, Q2, ok2] = circle_intersection(base_right, L3, [px py], L4);
if ~ok2
    q1=[]; q2=[]; return;
end

% --- Try the 4 possible combinations to find valid solution ---
candidates = [
    P1; Q1;
    P1; Q2;
    P2; Q1;
    P2; Q2
];

found = false;
best_q1 = [];
best_q2 = [];

for i = 1:4
    P = candidates(2*i-1,:);
    Q = candidates(2*i,:);

    % Ensure passive links L2 and L4 can connect
    d = norm(P - Q);
    if d > L2 + L4 || d < abs(L2 - L4)
        continue;
    end

    % Elbow-up check: cross(j1->j2, j1->end)
    v = Q - P;
    w = [px py] - P;
    cross_val = v(1)*w(2) - v(2)*w(1);

    if cross_val > 0   % elbow up
        % Compute joint angles for this candidate
        candidate_q1 = atan2(P(2)-base_left(2),  P(1)-base_left(1));
        candidate_q2 = atan2(Q(2)-base_right(2), Q(1)-base_right(1));
        
        % --- Angle limits (0~180 deg) ---
        if candidate_q1 >= 0 && candidate_q1 <= pi && candidate_q2 >= 0 && candidate_q2 <= pi
            best_q1 = candidate_q1;
            best_q2 = candidate_q2;
            found = true;
            break;
        end
    end
end

if ~found
    q1=[]; q2=[]; 
    valid = false;
    return;
end

q1 = best_q1;
q2 = best_q2;
valid = true;
end


%% -------------------------- Circle intersection --------------------------
function [p1, p2, ok] = circle_intersection(c1, r1, c2, r2)
d = norm(c2 - c1);
if d > r1 + r2 || d < abs(r1 - r2)
    p1=[]; p2=[]; ok=false;
    return;
end

a = (r1^2 - r2^2 + d^2) / (2*d);
h = sqrt(max(r1^2 - a^2, 0));

mid = c1 + a * (c2 - c1) / d;
offset = h * [-(c2(2)-c1(2))/d, (c2(1)-c1(1))/d];

p1 = mid + offset;
p2 = mid - offset;
ok = true;
end
