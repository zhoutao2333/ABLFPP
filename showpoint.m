clc
clear

%% 数据初始化
height = double(imread('heightmap.png'));


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

% 添加10个新的非边界点
new_points = zeros(10, 2);
for i = 1:10
    % 生成2-114范围内的随机坐标（避免边界）
    x = randi([2, 114]);
    y = randi([2, 114]);
    new_points(i, :) = [x, y];
end

% 将新点添加到原有点集合中
waypoints = [waypoints; new_points];

num_points = size(waypoints,1);
heights = arrayfun(@(i) height(waypoints(i,2), waypoints(i,1)) , 1:num_points)';
points = [waypoints, heights];



%% 绘制路径
figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
surf(x1, y1, height), shading interp, colorbar
hold on



% 标记点
for i = 1:num_points
    %text(tsp_points(i,1), tsp_points(i,2), points(i,3), num2str(i), 'Color', 'black');
    plot3(waypoints(i,1) , waypoints(i,2) , points(i,3)+5, 'o',...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','cyan',...
                       'MarkerSize',10);
end




title('TaskPoints 20');
hold off;

