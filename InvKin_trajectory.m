function angles = InvKin_trajectory(trajectory, robot)
angles = zeros(length(trajectory), 2);
index = 1;
for point = trajectory
    [q1, p1, result] = InvKin_5link(point, robot);
    if result
        angles(index, :) = [q1, p1];
        index = index + 1;
    end
end
end