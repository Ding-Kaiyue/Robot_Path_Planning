function [x, y, dmin_x, dmin_y] = minDistInObstacle(n, o)
obs = [o(1) o(2) o(1)+o(3) o(2)+o(4)];
% 点在障碍物正右侧
if n(1) > obs(3) && n(2) > obs(2) && n(2) < obs(4)
    x = obs(3);
    y = n(2);
    dmin_x = n(1) - obs(3);
    dmin_y = 0;
% 点在障碍物正左侧
elseif n(1) < obs(1) && n(2) > obs(2) && n(2) < obs(4)
    x = obs(1);
    y = n(2);
    dmin_x = n(1) - obs(1);
    dmin_y = 0;
% 点在障碍物正上侧
elseif n(2) > obs(4) && n(1) > obs(1) && n(1) < obs(3)
    x = n(1);
    y = obs(4);
    dmin_x = 0;
    dmin_y = n(2) - obs(4);
% 点在障碍物正下侧
elseif n(2) < obs(2) && n(1) > obs(1) && n(1) < obs(3)
    x = n(1);
    y = obs(2);
    dmin_x = 0;
    dmin_y = n(2) - obs(2);
% 点在障碍物左上方
elseif n(1) < obs(1) && n(2) > obs(4)
    x = obs(1);
    y = obs(4);
    dmin_x = n(1) - obs(1);
    dmin_y = n(2) - obs(4);
% 点在障碍物右上方
elseif n(1) > obs(3) && n(2) > obs(4)
    x = obs(3);
    y = obs(4);
    dmin_x = n(1) - obs(3);
    dmin_y = n(2) - obs(4);
% 点在障碍物左下方
elseif n(1) < obs(1) && n(2) < obs(3)
    x = obs(1);
    y = obs(3);
    dmin_x = n(1) - obs(1);
    dmin_y = n(2) - obs(2);
% 点在障碍物右下方
elseif n(1) > obs(3) && n(2) < obs(2)
    x = obs(3);
    y = obs(2);
    dmin_x = n(1) - obs(3);
    dmin_y = n(2) - obs(2);
else
    x = 0;
    y = 0;
    dmin_x = 0;
    dmin_y = 0;
end