function [field, startposind, goalposind, costchart, fieldpointers] = Reset_G_S(field, ...
        startposind, goalposind, costchart, fieldpointers, New_startposind, New_goalposind)

    field(startposind) = 10;
    field(goalposind) = 10;
    field(New_startposind) = 0;
    field(New_goalposind) = 0;

    costchart(startposind) = NaN;
    costchart(New_startposind) = 0;

    fieldpointers{startposind} = '1';
    fieldpointers{goalposind} = '1';
    fieldpointers{New_startposind} = 'S';
    fieldpointers{New_goalposind} = 'G';

    startposind = New_startposind;
    goalposind = New_goalposind;
end