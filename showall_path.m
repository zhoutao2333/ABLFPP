clc
clear

%% 数据初始化
HeightData = double(imread('heightmap (8).png'));
Obstacle_map = HeightData;
Obstacle_map(:) = 0;

% 设定多个点 (点1, 2, 3, 4)
waypoints = [
    98,101;
    %left
    107,87
    104,78
    104,69
    107,42
    107,33
    104,24
    101,15
    98,7

    %mid
    65,96;
    62,87;    
    62,78; 
    62,69; 
    62,60; 
    59,51; 
    56,42;
    50,17;
    44,8;

    %right
    23,96
    26,87
    23,78
    23,69
    20,60
    20,33
    17,24
    14,15
    14,7
];

figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
surf(x1, y1, HeightData), shading interp, colorbar
hold on

for i = 45:80
    if mod(i,3) == 2
        plot3(i', 101',HeightData(101,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,101) = 1;
end

for i = 49:82
    if mod(i,3) == 2
        plot3(i', 92',HeightData(92,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,92) = 1;
end

for i = 49:78
    if mod(i,3) == 2
        plot3(i', 83',HeightData(83,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,83) = 1;
end


for i = 45:78
    if mod(i,3) == 2
        plot3(i', 74',HeightData(74,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,74) = 1;
end

for i = 45:78
    if mod(i,3) == 2
        plot3(i', 65',HeightData(65,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,65) = 1;
end

for i = 45:78
    if mod(i,3) == 2
        plot3(i', 56',HeightData(56,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,56) = 1;
end

for i = 45:70
    if mod(i,3) == 2
        plot3(i', 47',HeightData(47,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,47) = 1;
end

for i = 45:65
    if mod(i,3) == 2
        plot3(i', 38',HeightData(38,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,38) = 1;
end

for i = 45:62
    if mod(i,3) == 2
        plot3(i', 22',HeightData(22,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,22) = 1;
end

for i = 40:58
    if mod(i,3) == 2
        plot3(i', 13',HeightData(13,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,13) = 1;
end

for i = 35:53
    if mod(i,3) == 2
        plot3(i', 4',HeightData(4,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,4) = 1;
end

%左侧
for i = 99:113
    if mod(i,3) == 2
        plot3(i', 92',HeightData(92,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,92) = 1;
end

for i = 95:113
    if mod(i,3) == 2
        plot3(i', 83',HeightData(83,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,83) = 1;
end

for i = 95:113
    if mod(i,3) == 2
        plot3(i', 74',HeightData(74,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,74) = 1;
end

for i = 95:113
    if mod(i,3) == 2
        plot3(i', 65',HeightData(65,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,65) = 1;
end

for i = 100:113
    if mod(i,3) == 2
        plot3(i', 47',HeightData(47,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,47) = 1;
end

for i = 100:113
    if mod(i,3) == 2
        plot3(i', 38',HeightData(38,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,38) = 1;
end

for i = 103:113
    if mod(i,3) == 2
        plot3(i', 29',HeightData(29,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,29) = 1;
end

for i = 90:113
    if mod(i,3) == 2
        plot3(i', 20',HeightData(20,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,20) = 1;
end

for i = 85:113
    if mod(i,3) == 2
        plot3(i', 11',HeightData(11,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,11) = 1;
end

for i = 75:113
    if mod(i,3) == 2
        plot3(i', 3',HeightData(3,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,3) = 1;
end


%右侧
for i = 10:35
    if mod(i,3) == 2
        plot3(i', 101',HeightData(101,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,101) = 1;
end

for i = 10:35
    if mod(i,3) == 2
        plot3(i', 92',HeightData(92,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,92) = 1;
end

for i = 15:35
    if mod(i,3) == 2
        plot3(i', 83',HeightData(83,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,83) = 1;
end

for i = 12:33
    if mod(i,3) == 2
        plot3(i', 74',HeightData(74,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,74) = 1;
end

for i = 9:33
    if mod(i,3) == 2
        plot3(i', 65',HeightData(65,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,65) = 1;
end

for i = 9:33
    if mod(i,3) == 2
        plot3(i', 56',HeightData(56,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,56) = 1;
end

for i = 9:33
    if mod(i,3) == 2
        plot3(i', 38',HeightData(38,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,38) = 1;
end

for i = 6:33
    if mod(i,3) == 2
        plot3(i', 29',HeightData(29,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,29) = 1;
end


for i = 5:31
    if mod(i,3) == 2
        plot3(i', 20',HeightData(20,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,20) = 1;
end

for i = 5:25
    if mod(i,3) == 2
        plot3(i', 11',HeightData(11,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,11) = 1;
end

for i = 5:22
    if mod(i,3) == 2
        plot3(i', 3',HeightData(3,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(i,3) = 1;
end

hold on


num_points = size(waypoints,1);
heights = arrayfun(@(i) HeightData(waypoints(i,2), waypoints(i,1)) , 1:num_points)';
points = [waypoints, heights];

% 计算所有点对 (i, j) 之间的最优路径

%% 绘制路径

tic;
[paths, costs] = IAstar_History_obs(HeightData,Obstacle_map, waypoints);
toc;

for i = 1:num_points
    for j = i+1:num_points
        path = paths{i, j};
        if ~isempty(path)
            height = arrayfun(@(idx) HeightData(path(idx,2), path(idx,1)), 1:size(path,1));
            plot3(path(:,1), path(:,2), height + 5, '-', 'LineWidth', 1.5, 'Color', 'red');
        end
    end
end

% 标记点
for i = 1:num_points
    if i == 1
        plot3(waypoints(i,1), waypoints(i,2), points(i,3)+5, 'rp',...
                           'MarkerEdgeColor','none',...
                           'MarkerFaceColor','r',...
                           'MarkerSize',10);       
        % 在第一个点旁边显示编号
        text(waypoints(i,1), waypoints(i,2), points(i,3)+5, num2str(i), ...
             'Color', 'blue', 'FontSize', 12, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    else
        plot3(waypoints(i,1), waypoints(i,2), points(i,3)+5, 'o',...
                           'MarkerEdgeColor','none',...
                           'MarkerFaceColor','cyan',...
                           'MarkerSize',10);
        % 在其他点旁边显示编号
        text(waypoints(i,1), waypoints(i,2)+3, points(i,3)+5, num2str(i), ...
             'Color', 'blue', 'FontSize', 10, ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
end


%title('TaskPoints 20');
hold off;

