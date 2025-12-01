function draw_5link(robot, robot_plot, q1, q2)

joints = ForwKin_5link(q1, q2, robot);

L0_plot = robot_plot.L0_plot;
L1_plot = robot_plot.L1_plot;
L2_plot = robot_plot.L2_plot;
L3_plot = robot_plot.L3_plot;
L4_plot = robot_plot.L4_plot;

base_point = robot_plot.base_point;
joint1 = robot_plot.joint1;
joint2 = robot_plot.joint2;
end_effector = robot_plot.end_effector;

if length(joints) < 5
    set(L1_plot, 'XData', NaN, 'YData', NaN);
    set(L2_plot, 'XData', NaN, 'YData', NaN);
    set(L3_plot, 'XData', NaN, 'YData', NaN);
    set(L4_plot, 'XData', NaN, 'YData', NaN);
    set(L0_plot, 'XData', NaN, 'YData', NaN);
    return;
end

base_left = joints(1, :);
base_right = joints(2, :);
joint1_pos = joints(3, :);
joint2_pos = joints(4, :);
end_effector_pos = joints(5, :);

set(L0_plot, 'XData', [base_left(1), base_right(1)], ...
    'YData', [base_left(2), base_right(2)]);

set(L1_plot, 'XData', [base_left(1), joint1_pos(1)], ...
    'YData', [base_left(2), joint1_pos(2)]);

set(L3_plot, 'XData', [base_right(1), joint2_pos(1)], ...
    'YData', [base_right(2), joint2_pos(2)]);

set(L2_plot, 'XData', [joint1_pos(1), end_effector_pos(1)], ...
    'YData', [joint1_pos(2), end_effector_pos(2)]);

set(L4_plot, 'XData', [joint2_pos(1), end_effector_pos(1)], ...
    'YData', [joint2_pos(2), end_effector_pos(2)]);

set(base_point, 'XData', [base_left(1), base_right(1)], ...
    'YData', [base_left(2), base_right(2)]);
set(joint1, 'XData', joint1_pos(1), 'YData', joint1_pos(2));
set(joint2, 'XData', joint2_pos(1), 'YData', joint2_pos(2));
set(end_effector, 'XData', end_effector_pos(1), 'YData', end_effector_pos(2));

end