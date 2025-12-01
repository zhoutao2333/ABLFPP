clc
clear
%%===================使用dijkstra + 贪心策略
%% 数据初始化
HeightData = double(imread('heightmap.png'));

% 设定多个点 (点1, 2, 3, 4)
waypoints = [
    101,101;

    100,9;
    101,28;
    101,42;
    102,57;
    103,71;
    102,84;

    85,7;
    84,20;
    85,34;
    83,46;
    84,60;
    83,72;
    85,85;
    83,95;

    71,7;
    70,20;
    71,32;
    69,46;
    70,60;
    69,75;
    70,90;

    53,5;
    55,17;
    55,32;
    56,46;
    55,61;
    56,76
    55,92;

    39,5;
    40,14;
    41,26;
    39,40;
    42,52;
    40,64;
    39,77;
    40,92;
    
    25,4;
    27,18;
    26,35;
    26,48;
    28,64;
    26,79;
    27,96;

    10,4;
    11,18;
    12,36;
    12,52;
    13,65;
    15,82;
    13,96;
];


num_points = size(waypoints,1);
heights = arrayfun(@(i) HeightData(waypoints(i,2), waypoints(i,1)) , 1:num_points)';
points = [waypoints, heights];

%使用Dijkstra求解所有路径
tic;
[paths, costs, len] = TSP_Dijkstra(HeightData, waypoints);
toc;


%使用贪心策略计算调度优化
tic
[Alpha_pos, Alpha_score] = TSP_Greedy(costs, num_points);
toc

alpha_len = 0;
for k = 1:num_points-1
    alpha_len = alpha_len + len(Alpha_pos(k), Alpha_pos(k+1));
end
% 添加回到起点的代价
alpha_len = alpha_len + len(Alpha_pos(num_points), Alpha_pos(1));
disp("length" + alpha_len)

disp(num2str(Alpha_score))

disp(Alpha_pos)
figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
surf(x1, y1, HeightData), shading interp, colorbar
hold on

% 绘制所有路径
BestSol = Alpha_pos;  % 获取最优解顺序
for i = 1:num_points
    if i == num_points
    path = paths{BestSol(i), BestSol(1)};   
    else
    path = paths{BestSol(i), BestSol(i+1)};
    end
    if ~isempty(path)
        height = arrayfun(@(idx) HeightData(path(idx,2), path(idx,1)), 1:size(path,1));
        plot3(path(:,1), path(:,2), height + 5, '-', 'LineWidth', 2,'Color', 'red');
    end
end

% 标记点
plot3(waypoints(1,1), waypoints(1,2), points(1,3)+6, 'rp',...
                   'MarkerEdgeColor','none',...
                   'MarkerFaceColor','r',...
                   'MarkerSize',10);       
% 在第一个点旁边显示编号
text(waypoints(1,1), waypoints(1,2), points(1,3)+6, "w", ...
     'Color', 'blue', 'FontSize', 12, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
disp(Alpha_pos)
for i = 2:length(Alpha_pos) - 1
    plot3(waypoints(Alpha_pos(i),1), waypoints(Alpha_pos(i),2), points(Alpha_pos(i),3)+6, 'o',...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','cyan',...
                       'MarkerSize',10);
    % 在其他点旁边显示编号
    text(waypoints(Alpha_pos(i),1), waypoints(Alpha_pos(i),2)+3, points(Alpha_pos(i),3)+6, num2str(i), ...
         'Color', 'blue', 'FontSize', 10, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

title('TaskPoints 20');
hold off;

