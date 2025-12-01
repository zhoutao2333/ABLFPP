clc
clear

%% 数据初始化
HeightData = double(imread('heightmap.png'));


Obstacle_map = HeightData;
Obstacle_map(:) = 0;
%94 89
%32 9
%endx=53;endy=89;endh = HeightData(endy,endx);
% 起点终点网格点 
startx=65;starty=87;starth = HeightData(starty,startx);
endx=21;endy=35;endh = HeightData(endy,endx);
% 最佳路径搜索

%% 绘制路径

figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
surf(x1, y1, HeightData), shading interp, colorbar

hold on;

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

tic
bestpath = IAstar_obs(HeightData,Obstacle_map, startx, starty, endx, endy);
toc

for i=1:size(bestpath,1)
    a(i,1)=bestpath(i,1);
    a(i,2)=bestpath(i,2);
end
disp("size" + size(bestpath,1));

plot3(a(1,1)', a(1,2)',HeightData(a(1,2),a(1,1)) + 10,'rp','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','r',...
                       'MarkerSize',10)
plot3(a(end,1)',a(end,2)',HeightData(a(end,2),a(end,1)) + 10,'bo','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','b',...
                       'MarkerSize',10)
%text(a(1,1)',a(1,2)',height(a(1,2),a(1,1)) + 10,'S');
%text(a(end,1)',a(end,2)',height(a(end,2),a(end,1)) + 10,'T');
xlabel('m','fontsize',12);
ylabel('m','fontsize',12);
zlabel('m','fontsize',12);
%title('Ours','fontsize',12)
set(gcf, 'Renderer', 'ZBuffer')
hold on
for i=1:size(a,1)
    high(i) = HeightData(a(i,2),a(i,1));
end
plot3(a(:,1)',a(:,2)',high + 10,'-','LineWidth',3)

%title('Map Build')
xlabel('x')
ylabel('y')