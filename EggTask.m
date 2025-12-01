% step
% 0: noting
% 1: wating
% 2: begining
% 3: checking
classdef EggTask < handle
    properties
        from
        to
        step
        robot
        trajectory
        angles
        anglesIndex
        createTime
    end
    
    methods
        function obj = EggTask(from, to, robot)
            if nargin > 0
                obj.from = from;
                obj.to = to;
                obj.step = 0;
                obj.robot = robot;
                
                x_traj = linspace(from(1), to(1), 20);
                y_traj = linspace(from(2), to(2), 20);
                traj = [x_traj; y_traj];
                obj.trajectory = traj;
                obj.angles = InvKin_trajectory(traj, robot);
                obj.anglesIndex = 1;
                
                obj.createTime = posixtime(datetime('now'));
            end
        end

        function begin(obj, currentQ1, currentQ2)
            joints = ForwKin_5link(currentQ1, currentQ2, obj.robot);
            currentPos = joints(5, :);
            x_traj = linspace(currentPos(1), obj.from(1), 20);
            y_traj = linspace(currentPos(2), obj.from(2), 20);
            traj = [x_traj; y_traj];
            obj.angles = [InvKin_trajectory(traj, obj.robot); obj.angles];
            obj.step = 2;
        end
        
        function [q1, q2] = move(obj, bot_plot)
            i = obj.anglesIndex;
            q1 = obj.angles(i,1);
            q2 = obj.angles(i,2);
            draw_5link(obj.robot, bot_plot, q1, q2);
            obj.anglesIndex = i + 1;
            if obj.anglesIndex > size(obj.angles, 1)
                obj.step = 3;
            end
        end

        function beginCheck(obj, egg)
            if egg.valid
                distance = norm(obj.from - egg.center);
                ok = distance < egg.radius / 2;
                if ~ok
                    obj.step = 0;
                end
            else
                obj.step = 0;
            end
        end

        function resultCheck(obj, ok)
            if ok
                obj.step = 0;
            end
        end
        
        function time = countdownTime(obj)
            time = 5 - floor(posixtime(datetime('now')) - obj.createTime);
        end
    end
end