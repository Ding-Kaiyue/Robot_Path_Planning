 function [field, startposind, goalposind, costchart, fieldpointers] = ...
     initializeField(n,wallpercent)
     %生成一个n*n的值为10的矩阵， 矩阵各元素表示两个单元格之间的距离
     field = 10*ones(n,n);
     %向上取整
     field(ind2sub([n n],ceil(n^2.*rand(n*n*wallpercent,1)))) = Inf;
     
     %% 随机生成起始点和终止点
     %随机生成起始点的索引值
     startposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));  
     %随机生成终止点的索引值
     goalposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));  
     %把矩阵中起始点和终止点处的值设为0
     field(startposind) = 0; field(goalposind) = 0;

     %生成一个nxn的矩阵costchart，每个元素都设为NaN。就是矩阵初始NaN无效数据
     costchart = NaN*ones(n,n);
     %在矩阵costchart中将起始点位置处的值设为0
     costchart(startposind) = 0;
    
     %% 生成元胞数组
     fieldpointers = cell(n,n);%生成元胞数组n*n
     % 将空白处表示为1
     fieldpointers(:) = {'1'};
     %将元胞数组的起始点的位置处设为 'S'，终止点处设为'G'
     fieldpointers{startposind} = 'S'; 
     fieldpointers{goalposind} = 'G'; 
     % 将障碍物表示为0
     fieldpointers(field==inf)={0};
 end
