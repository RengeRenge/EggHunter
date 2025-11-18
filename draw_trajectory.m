function draw_trajectory(ax, robot, traj)

angles = InvKin_trajectory(traj, robot);
num_points = size(angles, 1);

j_traj = zeros(num_points, 2, 3);
index = 1;

robot.delete_plot(ax);
robot_plot = robot.create_plot(ax);

for i = 1:num_points
    q1 = angles(i,1);
    q2 = angles(i,2);
    joints = ForwKin_5link(q1, q2, robot);
    j_traj(index, :, 1) = joints(3, :);
    j_traj(index, :, 2) = joints(4, :);
    j_traj(index, :, 3) = joints(5, :);
    index = index + 1;

    draw_5link(robot, robot_plot, q1, q2);

    left_joint = j_traj(:, :, 1);
    right_joint = j_traj(:, :, 2);
    end_joint = j_traj(:, :, 3);
    
    objs = findobj(ax, 'Tag', 'trajectory');
    if ~isempty(objs)
        delete(objs);
    end
    plot(ax, left_joint(:,1), left_joint(:,2), 'ko', 'MarkerSize', 1, 'MarkerFaceColor', 'k', 'Tag', 'trajectory');
    plot(ax, right_joint(:,1), right_joint(:,2), 'ko', 'MarkerSize', 1, 'MarkerFaceColor', 'k', 'Tag', 'trajectory');
    plot(ax, end_joint(:,1), end_joint(:,2), 'ko', 'MarkerSize', 1, 'MarkerFaceColor', 'k', 'Tag', 'trajectory');
    pause(0.1);
end
end