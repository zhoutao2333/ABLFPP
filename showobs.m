clc
clear

% 数据初始化
HeightData = double(imread('heightmap.png'));
Obstacle_map = HeightData;
Obstacle_map(:) = 0;


disp(HeightData)
% 起点终点网格点 

figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
disp(class(HeightData))
disp(HeightData(2,3))
surf(x1, y1, HeightData), shading interp, colorbar
hold on

%中间
for i = 45:80
    if mod(i,3) == 2
        plot3(i', 101',HeightData(101,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 49:82
    if mod(i,3) == 2
        plot3(i', 92',HeightData(92,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 49:78
    if mod(i,3) == 2
        plot3(i', 83',HeightData(83,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end


for i = 45:78
    if mod(i,3) == 2
        plot3(i', 74',HeightData(74,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 45:78
    if mod(i,3) == 2
        plot3(i', 65',HeightData(65,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 45:78
    if mod(i,3) == 2
        plot3(i', 56',HeightData(56,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 45:70
    if mod(i,3) == 2
        plot3(i', 47',HeightData(47,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 45:65
    if mod(i,3) == 2
        plot3(i', 38',HeightData(38,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 45:62
    if mod(i,3) == 2
        plot3(i', 22',HeightData(22,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 40:58
    if mod(i,3) == 2
        plot3(i', 13',HeightData(13,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 35:53
    if mod(i,3) == 2
        plot3(i', 4',HeightData(4,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
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
    Obstacle_map(101,i) = 1;
end

for i = 95:113
    if mod(i,3) == 2
        plot3(i', 83',HeightData(83,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 95:113
    if mod(i,3) == 2
        plot3(i', 74',HeightData(74,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 95:113
    if mod(i,3) == 2
        plot3(i', 65',HeightData(65,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 100:113
    if mod(i,3) == 2
        plot3(i', 47',HeightData(47,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 100:113
    if mod(i,3) == 2
        plot3(i', 38',HeightData(38,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 103:113
    if mod(i,3) == 2
        plot3(i', 29',HeightData(29,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 90:113
    if mod(i,3) == 2
        plot3(i', 20',HeightData(20,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 85:113
    if mod(i,3) == 2
        plot3(i', 11',HeightData(11,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 75:113
    if mod(i,3) == 2
        plot3(i', 3',HeightData(3,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
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
    Obstacle_map(101,i) = 1;
end

for i = 10:35
    if mod(i,3) == 2
        plot3(i', 92',HeightData(92,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 15:35
    if mod(i,3) == 2
        plot3(i', 83',HeightData(83,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 12:33
    if mod(i,3) == 2
        plot3(i', 74',HeightData(74,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 9:33
    if mod(i,3) == 2
        plot3(i', 65',HeightData(65,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 9:33
    if mod(i,3) == 2
        plot3(i', 56',HeightData(56,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 9:33
    if mod(i,3) == 2
        plot3(i', 38',HeightData(38,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 6:33
    if mod(i,3) == 2
        plot3(i', 29',HeightData(29,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 5:33
    if mod(i,3) == 2
        plot3(i', 29',HeightData(29,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 5:31
    if mod(i,3) == 2
        plot3(i', 20',HeightData(20,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 5:25
    if mod(i,3) == 2
        plot3(i', 11',HeightData(11,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

for i = 5:22
    if mod(i,3) == 2
        plot3(i', 3',HeightData(3,i)+5,'bo','MarkerSize',5,...
                               'MarkerEdgeColor','none',...
                               'MarkerFaceColor','g',...
                               'MarkerSize',6)
        set(gca, 'LooseInset', [0,0,0,0]);
    end
    Obstacle_map(101,i) = 1;
end

hold on

title('山地果园')
xlabel('x')
ylabel('y')