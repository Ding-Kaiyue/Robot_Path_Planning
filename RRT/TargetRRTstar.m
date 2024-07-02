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

for i = 1:numNodes
    if rand > 0 && rand < 0.2
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

    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    if noCollision(q_rand, q_near.coord, obstacle1) ...
    && noCollision(q_rand, q_near.coord, obstacle2) ...
    && noCollision(q_rand, q_near.coord, obstacle3)
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