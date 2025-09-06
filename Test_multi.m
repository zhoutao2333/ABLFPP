clc
clear

%% 数据初始化
HeightData = double(imread('heightmap (8).png'));

% 设定多个点 (点1, 2, 3, 4)
waypoints = [
    90,91;
    3, 4;  
    54, 87; 
    14, 84;  
    77, 27;   

    35,34;
    42,64;
    14,47;
    41,20;
    88,52;

    72,50;
    64,18;
    35,88;
    15,24;
    93,69;

    5,66;
    26,15;
    35,52;
    63,65;
    89,21;
];




num_points = size(waypoints,1);
heights = arrayfun(@(i) HeightData(waypoints(i,2), waypoints(i,1)) , 1:num_points)';
points = [waypoints, heights];

% 计算所有点对 (i, j) 之间的最优路径

%% 绘制路径


tic;
[paths, costs, len] = IAstar_History(HeightData, waypoints);
toc;

disp(num2str(costs))
tic
[Alpha_pos, Alpha_score] = MSD_GWO(costs, num_points, 50, 100);
toc

alpha_len = 0;
for k = 1:num_points-1
    alpha_len = alpha_len + len(Alpha_pos(k), Alpha_pos(k+1));
end

alpha_len = alpha_len + len(Alpha_pos(num_points), Alpha_pos(1));

disp(Alpha_score)

disp(Alpha_pos)
figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
surf(x1, y1, HeightData), shading interp, colorbar
hold on

% 绘制所有路径
BestSol = Alpha_pos;  % 获取最优解顺序
for i = 1:num_points-1
    path = paths{BestSol(i), BestSol(i+1)};
    if ~isempty(path)
        height = arrayfun(@(idx) HeightData(path(idx,2), path(idx,1)), 1:size(path,1));
        plot3(path(:,1), path(:,2), height + 5, '-', 'LineWidth', 2,'Color', 'red');
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

title('TaskPoints 20');
hold off;

