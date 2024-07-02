clc;
clear all;
close all;
%% A* 环境创建
n = 50;
wallpersent = 0.45;
Weights = 2;        % h(n)的权重
Corner_amend = 1;   % 1-进行拐角优化  0-不进行拐角优化
Environment_Set = 0; % 0-使用上一次创建的环境信息, 设定为1, 重新随机生成障碍物
Reset_GS = 1;       % 1-重设起始点和终止点   0-关闭
New_startposind = 2246; 
New_goalposind = 2313;
Road_Length = 0;    % 路径长度
% 方格及障碍物创建
if (Environment_Set)
    [field, startposind, goalposind, costchart, fieldpointers] = initializeField(n, wallpersent);
    save('Environmental', 'field', 'startposind', 'goalposind', 'costchart', 'fieldpointers')
else
    load('Environmental');
end

% 重设起始点和终止点
if (Reset_GS)
    [field, startposind, goalposind, costchart, fieldpointers] = Reset_G_S(field, ...
        startposind, goalposind, costchart, fieldpointers, New_startposind, New_goalposind);
end

axishandle = createFigure(field, costchart, startposind, goalposind);
%% 路径修正变量
Parent_node = 0;
Expected_node = 0;
untext_ii = 0;
ament_count = 0;    % 修正次数
%% A* 路径规划
setOpen = [startposind]; setOpenCosts = [0]; setOpenHeuristics = [Inf];
setClosed = []; setClosedCosts = [];
movementdirections = {'R', 'L', 'D', 'U'};
%% 
while ~max(ismember(setOpen,goalposind)) && ~isempty(setOpen)
    % 将待选子节点中最优的一个放入setClosed,  并将其从setOpen中删除
    [temp, ii] = min(setOpenCosts + Weights * setOpenHeuristics);
    % 拐角优化
    if ((setOpen(ii) ~= startposind) && (Corner_amend == 1))
        % 路径修正, 在不增加距离的基础上，减少转弯次数
        [new_ii, ament_count_1] = Path_optimization(temp, ii, fieldpointers, ...
            setOpen, setOpenCosts, startposind, Weights, setOpenHeuristics, ...
            Parent_node, Expected_node, untext_ii, ament_count);
        ii = new_ii;
        ament_count = ament_count_1;
    end

    [costs, heuristics, posinds] = findFValue(setOpen(ii), setOpenCosts(ii), ...
        field, goalposind, 'euclidean');
    setClosed = [setClosed; setOpen(ii)];
    setClosedCosts = [setClosedCosts; setOpenCosts(ii)];

    if (ii > 1 && ii < length(setOpen))      
        setOpen = [setOpen(1 : ii - 1); setOpen(ii + 1 : end)];
        setOpenCosts = [setOpenCosts(1 : ii - 1); setOpenCosts(ii + 1 : end)];
        setOpenHeuristics = [setOpenHeuristics(1 : ii -1); setOpenHeuristics(ii + 1 : end)];
    elseif (ii == 1)
        setOpen = setOpen(2 : end);
        setOpenCosts = setOpenCosts(2: end);
        setOpenHeuristics = setOpenHeuristics(2: end);
    else
        setOpen = setOpen(1 : end - 1);
        setOpenCosts = setOpenCosts(1: end - 1);
        setOpenHeuristics = setOpenHeuristics(1: end - 1);
    end
    % 将拓展出来的点中符合要求的点放到setOpen矩阵中, 作为待选点
    for jj = 1 : length(posinds)
        if ~isinf(costs(jj))
            % 拓展点不再setOpen中也不在setClosed中, 加入setOpen
            if ~max([setClosed; setOpen] == posinds(jj))
                fieldpointers(posinds(jj)) = movementdirections(jj);
                costchart(posinds(jj)) = costs(jj);
                setOpen = [setOpen; posinds(jj)];
                setOpenCosts = [setOpenCosts; costs(jj)];
                setOpenHeuristics = [setOpenHeuristics; heuristics(jj)];
    
            % 拓展点在setOpen中, 判断是否比原路径的cost小, 是则替换, 否则不处理
            elseif max(setOpen == posinds(jj))
                I = find(setOpen == posinds(jj));
                if setOpenCosts(I) > costs(jj)
                    costchart(setOpen(I)) = costs(jj);
                    setOpenCosts(I) = costs(jj);
                    setOpenHeuristics(I) = heuristics(jj);
                    fieldpointers(setOpen(I)) = movementdirections(jj);
                end
            % 拓展点在setClosed中
            else
                I = find(setClosed == posinds(jj));
                if setClosedCosts(I) > costs(jj)
                    costchart(setClosed(I)) = costs(jj);
                    setClosedCosts(I) = costs(jj);
                    fieldpointers(setClosed(I)) = movementdirections(jj);
                end
            end
        end
    end
    % 调整方格颜色
    if isempty(setOpen) 
        break; 
    end
    set(axishandle,'CData',[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);
    % hack to make image look right
    set(gca,'CLim',[0 1.1*max(costchart(find(costchart < Inf)))]);
    drawnow;
end
%% 通过路径回溯找出规划的路径
if max(ismember(setOpen, goalposind))
    disp('--Solution Found--');
    [p, Road_Length] = findWayBack(goalposind, fieldpointers, Road_Length);
    plot(p(:,2) + 0.5, p(:, 1) + 0.5, 'Color', 0.2*ones(3,1), 'LineWidth', 4);
    drawnow;
    drawnow;

elseif isempty(setOpen)
    disp('--No Solution--');
end