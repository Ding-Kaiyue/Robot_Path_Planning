% Algorithm Flow
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from explored nodes to branch out from,
%    towards q_rand.
% 3. Steer from q_near towards q_rand: interpolate if node is too far away,
%    reach q_new. Check that obstacle is not hit.
% 4. Update cost of reaching q_new from q_near, treat it as Cmin. For now,
%    q_near acts as the parent node of q_new.
% 5. From the list of 'visited' nodes, check for the nearest neighbors with
%    a given radius, insert in a list q_nearest.
% 6. In all members of q_nearest, check if q_new can be reached from a
%    different parent node with cost lower than Cmin, and without colliding
%    with the obstacle. Select the node that results in the least cost and
%    update the parent of q_new.
% 7. Add q_new to node list
% 8. Continue until maximum number of nodes is reached or goal is hit.
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
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    plot(q_rand(1), q_rand(2), 'x', 'Color', [0 0.4470 0.7410])

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

        q_nearest = [];
        r = 60;
        neighbor_count = 1;
        for j = 1:length(nodes)
            if noCollision(nodes(j).coord, q_new.coord, obstacle1) ...
            && noCollision(nodes(j).coord, q_new.coord, obstacle2) ...
            && noCollision(nodes(j).coord, q_new.coord, obstacle3) ...
            && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count + 1;
            end
        end

        q_min = q_near;
        C_min = q_new.cost;

        for k = 1:length(q_nearest)
            if noCollision(q_nearest(k).coord, q_new.coord, obstacle1) ...
            && noCollision(q_nearest(k).coord, q_new.coord, obstacle2) ...
            && noCollision(q_nearest(k).coord, q_new.coord, obstacle3) ...
            && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line ([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');
                hold on 
            end
        end

        for j = 1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end

        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes, q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodes(start);
end









