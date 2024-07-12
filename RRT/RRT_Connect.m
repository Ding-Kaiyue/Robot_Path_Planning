clearvars
close all
addpath('function')
% Define obstacles and environment boundaries
x_max = 1000;
y_max = 1000;
obstacle1 = [500, 450, 200, 200];
obstacle2 = [200, 600, 200, 200];
obstacle3 = [600, 150, 200, 200];

% Parameters
numNodes = 3000;
EPS = 20;

% Define start and goal configurations
q_start.coord = [0 0];
q_start.cost = 0;
q_start.parent = 0;

q_goal.coord = [999 999];
q_goal.cost = 0;
q_goal.parent = 0;

% Initialize trees
nodes1(1) = q_start;
nodes2(1) = q_goal;

% Plot obstacles
figure(1)
axis([0 x_max 0 y_max])
rectangle('Position', obstacle1, 'FaceColor', [0,.5,.5])
hold on
rectangle('Position', obstacle2, 'FaceColor', [0,.5,.5])
hold on
rectangle('Position', obstacle3, 'FaceColor', [0,.5,.5])
hold on

treeTurn = 1; % Start with Tree 1

for i = 1:numNodes
    if treeTurn == 1
        % Extend Tree 1
        q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
        plot(q_rand(1), q_rand(2), 'x', 'Color', [0 0.4470 0.7410]);
        
        % Find nearest node in Tree 1
        ndist1 = [];
        for j = 1:length(nodes1)
            tmp = dist(nodes1(j).coord, q_rand);
            ndist1 = [ndist1 tmp];
        end
        [val1, idx1] = min(ndist1);
        q_near1 = nodes1(idx1);

        q_new.coord = steer(q_rand, q_near1.coord, val1, EPS);
        if noCollision(q_new.coord, q_near1.coord, obstacle1) ...
           && noCollision(q_new.coord, q_near1.coord, obstacle2) ...
           && noCollision(q_new.coord, q_near1.coord, obstacle3)
            line([q_near1.coord(1), q_new.coord(1)], [q_near1.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
            drawnow
            hold on
            q_new.cost = dist(q_new.coord, q_near1.coord) + q_near1.cost;
            % q_new.parent = length(nodes1) + 1;
         
            q_new.parent = idx1;
            nodes1 = [nodes1 q_new];

            % Check connection to Tree 2
            ndist2 = [];
            for j = 1:length(nodes2)
                tmp = dist(nodes2(j).coord, q_new.coord);
                ndist2 = [ndist2 tmp];
            end
            [val2, idx2] = min(ndist2);
            q_near2 = nodes2(idx2);
            
            if dist(q_new.coord, q_near2.coord) < EPS ...
               && noCollision(q_new.coord, q_near2.coord, obstacle1) ...
               && noCollision(q_new.coord, q_near2.coord, obstacle2) ...
               && noCollision(q_new.coord, q_near2.coord, obstacle3)
                line([q_new.coord(1), q_near2.coord(1)], [q_new.coord(2), q_near2.coord(2)], 'Color', 'k', 'LineWidth', 2);
                drawnow
                hold on
                break; % Connection found
            end
        end
        
        treeTurn = 2; % Switch to Tree 2 next
    else
        % Extend Tree 2
        q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
        plot(q_rand(1), q_rand(2), 'x', 'Color', [0 0.4470 0.7410]);
        % Find nearest node in Tree 2
        ndist2 = [];
        for j = 1:length(nodes2)
            tmp = dist(nodes2(j).coord, q_rand);
            ndist2 = [ndist2 tmp];
        end
        [val2, idx2] = min(ndist2);
        q_near2 = nodes2(idx2);

        q_new.coord = steer(q_rand, q_near2.coord, val2, EPS);
        if noCollision(q_new.coord, q_near2.coord, obstacle1) ...
           && noCollision(q_new.coord, q_near2.coord, obstacle2) ...
           && noCollision(q_new.coord, q_near2.coord, obstacle3)
            line([q_near2.coord(1), q_new.coord(1)], [q_near2.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
            drawnow
            hold on
            q_new.cost = dist(q_new.coord, q_near2.coord) + q_near2.cost;
            % q_new.parent = length(nodes2) + 1;
            
            q_new.parent = idx2;
                
            nodes2 = [nodes2 q_new];

            % Check connection to Tree 1
            ndist1 = [];
            for j = 1:length(nodes1)
                tmp = dist(nodes1(j).coord, q_new.coord);
                ndist1 = [ndist1 tmp];
            end
            [val1, idx1] = min(ndist1);
            q_near1 = nodes1(idx1);
            
            if dist(q_new.coord, q_near1.coord) < EPS ...
               && noCollision(q_new.coord, q_near1.coord, obstacle1) ...
               && noCollision(q_new.coord, q_near1.coord, obstacle2) ...
               && noCollision(q_new.coord, q_near1.coord, obstacle3)
                line([q_new.coord(1), q_near1.coord(1)], [q_new.coord(2), q_near1.coord(2)], 'Color', 'k', 'LineWidth', 2);
                drawnow
                hold on
                break; % Connection found
            end
        end
        
        treeTurn = 1; % Switch to Tree 1 next
    end
end

q_end1 = q_near1;
q_end2 = q_near2;

line([q_end1.coord(1), q_new.coord(1)], [q_end1.coord(2), q_new.coord(2)], 'Color','r', 'LineWidth', 2);
line([q_end2.coord(1), q_new.coord(1)], [q_end2.coord(2), q_new.coord(2)], 'Color','r', 'LineWidth', 2);
% Reconstruct path for Tree 1
while q_end1.parent ~= 0
    start = q_end1.parent;
    line([q_end1.coord(1), nodes1(start).coord(1)], [q_end1.coord(2), nodes1(start).coord(2)], 'Color','r', 'LineWidth', 2);
    drawnow
    hold on
    q_end1 = nodes1(start);
end
 
% Reconstruct path for Tree 2
while q_end2.parent ~= 0
    start = q_end2.parent;
    line([q_end2.coord(1), nodes2(start).coord(1)], [q_end2.coord(2), nodes2(start).coord(2)], 'Color','r', 'LineWidth', 2);
    drawnow
    hold on
    q_end2 = nodes2(start);
end