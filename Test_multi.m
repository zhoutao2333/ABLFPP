clc
clear

% 数据初始化
HeightData = double(imread('heightmap.png'));

% 设定多个点 (点1, 2, 3, 4)
waypoints = [
    101,101;
    100,9;
    101,32;
    101,49;
    99,66;
    103,83;

    80,10;
    80,29;
    79,46;
    80,64;
    78,83;

    60,8;
    61,23;
    62,42;
    62,62;
    61,90;

    42,7;
    44,25;
    42,43;
    43,64;
    45,87

    25,9;
    27,44;
    29,68;
    30,29;
    32,87;

    9,10;
    10,29;
    11,46;
    9,70;
    14,88
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
[Alpha_pos, Alpha_score] = MSD_GWO(costs, num_points, 100, 100);
toc

total_e = 0;
total_l = 0;
%% 循环求解每两点间路径
allpath = [];   % 存放完整路径

tic
for k = 1:length(Alpha_pos) - 1
    waypoints_tem = [
        waypoints(Alpha_pos(k),1), waypoints(Alpha_pos(k),2);
        waypoints(Alpha_pos(k+1),1), waypoints(Alpha_pos(k+1),2);
    ];
    disp(k)
    disp("起点：" + Alpha_pos(k) + " 终点：" + Alpha_pos(k+1))
    % 调用A*能量最优路径规划函数
    [path, costA, path_length] = Improved_Astar(HeightData, waypoints_tem);
    total_e = total_e + costA;
    total_l = total_l + path_length;
    % 拼接路径（避免重复）
    if k == 1
        allpath = path;
    else
        allpath = [allpath; path(2:end,:)]; % 去掉重复的首点
    end
end

waypoints_tem = [
    waypoints(Alpha_pos(length(Alpha_pos)),1), waypoints(Alpha_pos(length(Alpha_pos)),2);
    waypoints(Alpha_pos(1),1), waypoints(Alpha_pos(1),2);
];

[path, costA, path_length] = Improved_Astar(HeightData, waypoints_tem);
total_e = total_e + costA;
total_l = total_l + path_length;
allpath = [allpath; path(2:end-1,:)]; % 去掉重复的首点
toc
disp("total_e:" + total_e);
disp("total_l:" + total_l);
%% 绘制路径
a = allpath;
disp("总路径点数: " + size(a,1));

figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
surf(x1, y1, HeightData), shading interp, colorbar
hold on

for i=1:size(a,1)
    high(i) = HeightData(a(i,2),a(i,1));
end
plot3(a(:,1)', a(:,2)', high + 3, 'r-', 'LineWidth', 2)

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

