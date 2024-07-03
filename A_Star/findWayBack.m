function [p, Road_Length] = findWayBack(goalposind, fieldpointers, Road_Length)
    n = length(fieldpointers);
    posind = goalposind;
    [py, px] = ind2sub([n n], posind);
    p = [py, px];

    while ~strcmp(fieldpointers{posind}, 'S')
        switch fieldpointers{posind}
            case 'L' 
                px = px - 1;
            case 'R'
                px = px + 1;
            case 'U'
                py = py - 1;
            case 'D'
                py = py + 1;
        end
        p = [p; py px];
        posind = sub2ind([n n], py, px);
        Road_Length = Road_Length + 1;
    end
end
