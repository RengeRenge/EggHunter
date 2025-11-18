function bot = robot()
bot.L0 = 20;
bot.L1 = 30;
bot.L2 = 30;
bot.L3 = 25;
bot.L4 = 30;
bot.scale = 1;
bot.base = [0,0];

bot.create_plot = @create_plot;
bot.delete_plot = @delete_plot;

end

function p = create_plot(main_ax)
p.L0_plot = plot(main_ax, NaN, NaN, 'k-', 'LineWidth', 8, 'Tag', 'robot');
p.L1_plot = plot(main_ax, NaN, NaN, 'r-', 'LineWidth', 3, 'Tag', 'robot');
p.L2_plot = plot(main_ax, NaN, NaN, 'b-', 'LineWidth', 3, 'Tag', 'robot');
p.L3_plot = plot(main_ax, NaN, NaN, 'g-', 'LineWidth', 3, 'Tag', 'robot');
p.L4_plot = plot(main_ax, NaN, NaN, 'm-', 'LineWidth', 3, 'Tag', 'robot');

p.base_point = plot(main_ax, NaN, NaN, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'Tag', 'robot');
p.joint1 = plot(main_ax, NaN, NaN, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'Tag', 'robot');
p.joint2 = plot(main_ax, NaN, NaN, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g', 'Tag', 'robot');
p.end_effector = plot(main_ax, NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'Tag', 'robot');

end

function delete_plot(main_ax)
objs = findobj(main_ax, 'Tag', 'robot');
if ~isempty(objs)
    delete(objs);
end
end


% % config = [24, 36, 36, 30, 36];
% config = [20, 30, 30, 25, 30];
% 
% % 任意目标点
% px = -30.9195; py = 18.8903;
% 
% % 计算逆解
% [q1, q2] = InvKin_5link(0, 0, [px, py], config, 1);
% 
% % 再用正解算回末端
% joints = ForwKin_5link(0, 0, config, q1, q2, 1);
% 
% end_fk = joints(5,:);   % 你 ForwKin 中 end_effector_pos 是第四行
% 
% fprintf("期望末端: (%.4f, %.4f)\n", px, py);
% fprintf("FK 返回末端: (%.4f, %.4f) q1 q2: (%.4f, %.4f)\n", end_fk(1), end_fk(2), q1, q2);
