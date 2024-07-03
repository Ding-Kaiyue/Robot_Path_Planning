function [cost, heuristic, posinds] = findFValue(posind, costsofar, field, ...
          goalind, heuristicmethod)
    n = length(field);
    % 将父节点和终止点索引值拓展成坐标值
    [currentpos(1), currentpos(2)] = ind2sub([n n], posind);
    [goalpos(1), goalpos(2)] = ind2sub([n n], goalind);
    % 4个方向, 所以是4行, pos是2列指一个点的位置用x和y表示
    cost = Inf * ones(4, 1);
    heuristic = Inf * ones(4, 1);
    pos = ones(4, 2);

    %% 方向1
    newx = currentpos(2) - 1;
    newy = currentpos(1);
    if newx > 0
        pos(1,:) = [newy newx];
        switch lower(heuristicmethod)
            case 'euclidean'
                heuristic(1) = 10*abs(goalpos(2) - newx) + 10*abs(goalpos(1) - newy);
            case 'taxicab'
                heuristic(1) = 10*abs(goalpos(2) - newx) + 10*abs(goalpos(1) - newy);
        end
        cost(1) = costsofar + field(newy, newx);
    end
    %% 方向2
    newx = currentpos(2) + 1;
    newy = currentpos(1);
    if newx <= n
        pos(2,:) = [newy newx];
        switch lower(heuristicmethod)
            case 'euclidean'
                heuristic(2) = 10*abs(goalpos(2) - newx) + 10*abs(goalpos(1) - newy);
            case 'taxicab'
                heuristic(2) = 10*abs(goalpos(2) - newx) + 10*abs(goalpos(1) - newy);
        end
        cost(2) = costsofar + field(newy, newx);
    end
    %% 方向3
    newx = currentpos(2);
    newy = currentpos(1) - 1;
    if newy > 0
        pos(3,:) = [newy newx];
        switch lower(heuristicmethod)
            case 'euclidean'
                heuristic(3) = 10*abs(goalpos(2) - newx) + 10*abs(goalpos(1) - newy);
            case 'taxicab'
                heuristic(3) = 10*abs(goalpos(2) - newx) + 10*abs(goalpos(1) - newy);
        end
        cost(3) = costsofar + field(newy, newx);
    end
    %% 方向4
    newx = currentpos(2);
    newy = currentpos(1) + 1;
    if newy <= n
        pos(4,:) = [newy newx];
        switch lower(heuristicmethod)
            case 'euclidean'
                heuristic(4) = 10*abs(goalpos(2) - newx) + 10*abs(goalpos(1) - newy);
            case 'taxicab'
                heuristic(4) = 10*abs(goalpos(2) - newx) + 10*abs(goalpos(1) - newy);
        end
        cost(4) = costsofar + field(newy, newx);
    end
    %% 
    posinds = sub2ind([n n], pos(:,1), pos(:,2));
end

    