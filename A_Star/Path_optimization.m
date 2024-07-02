function [new_ii, ament_count_1] = Path_optimization(temp, ii, fieldpointers, ...
    setOpen, setOpenCosts, startposind, Weights, setOpenHeuristics, Parent_node, ...
    Expected_node, untext_ii, ament_count)
    n = length(fieldpointers);

    switch fieldpointers {setOpen(ii)}
        case 'L'
            Parent_node = setOpen(ii) - n;
        case 'R'
            Parent_node = setOpen(ii) + n;
        case 'U'
            Parent_node = setOpen(ii) - 1;
        case 'D'
            Parent_node = setOpen(ii) + 1;
    end
    % 起始点跳过修正
    if Parent_node == startposind
        new_ii = ii;
        ament_count_1 = ament_count;
    else 
        % 获取期望下一步要走的点的索引值
        switch fieldpointers{Parent_node}
            case 'L'
                Expected_node = Parent_node + n;
            case 'R'
                Expected_node = Parent_node - n;
            case 'U'
                Expected_node = Parent_node + 1;
            case 'D'
                Expected_node = Parent_node - 1;
        end
    
        % 判断期望点是否在边界内
        if ((Expected_node <= 0) || (Expected_node > n*n))
            new_ii = ii;
            ament_count_1 = ament_count;
        else
            % 计算新的要进行拓展的点在setOpen中的索引值  
            if fieldpointers{setOpen(ii)} == fieldpointers{Parent_node}
                new_ii = ii;
                ament_count_1 = ament_count;
            elseif find(setOpen == Expected_node)
                untext_ii = find(setOpen == Expected_node)
                now_cost = setOpenCosts(untext_ii) + Weights * setOpenHeuristics(untext_ii);
                if temp == now_cost
                    new_ii = untext_ii;
                    ament_count = ament_count + 1;
                    ament_count_1 = ament_count;
                else
                    new_ii = ii;
                    ament_count_1 = ament_count;
                end
            else
                new_ii = ii;
                ament_count_1 = ament_count;
            end
        end
    end
end
