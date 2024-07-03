% Goal-bias RRT 

clearvars
close all
addpath('function')
x_max = 1000;
y_max = 1000;
obstacle1 = [500, 450, 200, 200];
obstacle2 = [200, 600, 200, 200];
obstacle3 = [600, 150, 200, 200];
numNodes = 3000;
EPS = 20;
Pmax = 0.2;
P = Pmax;

Falt_star = 5; % 引力常数
Frep_star = 7; % 斥力常数
drep_star = 40; % 斥力势场的接近半径
k = 5; % 形状系数
epsilon = 30; % 随机分量因子
phi = 25; % 势场分量因子
local_minima = 0; % 是否陷入局部最优解
n_star = 10; % 最大尝试次数

q_start.coord = [0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [999 999];
q_goal.cost = 0;

nodes(1) = q_start;
figure(1)
axis([0 x_max 0 y_max])
rectangle('Position', obstacle1, 'FaceColor', [0,.5,.5])
hold on
rectangle('Position', obstacle2, 'FaceColor', [0,.5,.5])
hold on
rectangle('Position', obstacle3, 'FaceColor', [0,.5,.5])
hold on

for i=1:numNodes
    if rand < P
        q_rand = q_goal.coord;
    else
        q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    end
    plot(q_rand(1), q_rand(2), 'x', 'Color', [0 0.4470 0.7410]);

    for j = 1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end

    ndist = [];
    for j = 1:length(nodes)
        tmp = dist(nodes(j).coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);

    [x1, y1, dmin1_x, dmin1_y] = minDistInObstacle(q_near.coord, obstacle1);
    [x2, y2, dmin2_x, dmin2_y] = minDistInObstacle(q_near.coord, obstacle2);
    [x3, y3, dmin3_x, dmin3_y] = minDistInObstacle(q_near.coord, obstacle3);
    d1_abs = ((dmin1_x)^2 + (dmin1_y)^2);
    d2_abs = ((dmin2_x)^2 + (dmin2_y)^2);
    d3_abs = ((dmin3_x)^2 + (dmin3_y)^2);
    distance = [d1_abs, d2_abs, d3_abs];
    if d1_abs == min(distance)
        o_near = [x1, y1];
        dmin_abs = d1_abs;
        dmin = [dmin1_x, dmin1_y];
    elseif d2_abs == min(distance)
        o_near = [x2, y2];
        dmin_abs = d2_abs;
        dmin = [dmin2_x, dmin2_y];
    else
        o_near = [x3, y3];
        dmin_abs = d3_abs;
        dmin = [dmin3_x, dmin3_y];
    end
    % 引力函数
    F_alt = Falt_star * (q_goal.coord - q_near.coord) / dist(q_goal.coord, q_near.coord);
    % 斥力函数
    if dmin_abs <= drep_star
        F_rep = Frep_star * dmin / dmin_abs / (1 + exp((2 * dmin_abs / drep_star - 1)*k));
    else
        F_rep = [0, 0];
    end
    F_total = F_rep + F_alt;
    q_new.coord = q_near.coord + epsilon * (q_rand - q_near.coord) / dist(q_rand, q_near.coord)...
          + phi * (F_total) / sqrt(F_total(1)^2 + F_total(2)^2);

    if noCollision(q_new.coord, q_near.coord, obstacle1)...
    && noCollision(q_new.coord, q_near.coord, obstacle2)...
    && noCollision(q_new.coord, q_near.coord, obstacle3)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        for j = 1:length(nodes)
            if nodes(j).coord == q_near.coord
                q_new.parent = j;
            end
        end
        nodes = [nodes q_new];

        if q_goal.coord(1) - q_new.coord(1) < EPS && q_goal.coord(2) - q_new.coord(2) < EPS
            line([q_new.coord(1), q_goal.coord(1)], [q_new.coord(2), q_goal.coord(2)], 'Color', 'k', 'LineWidth', 2);
            drawnow
            hold on
            for j = 1:length(nodes)
                if nodes(j).coord == q_new.coord
                    q_goal.parent = j;
                end
            end
            break
        end
        
        if q_rand == q_goal.coord
            P = Pmax;
            local_minima = false;
        end
    elseif q_rand == q_goal.coord
        local_minima = true;
        n = 0;
    end

    if local_minima
        P = Pmax * (1 - exp(-k*n/n_star));
        n = n + 1;
    end
end

q_end = q_goal;
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color','r', 'LineWidth', 2);
    drawnow
    hold on
    q_end = nodes(start);
end