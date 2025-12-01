% 使用z*结合MSD_GWO
HeightData = double(imread('heightmap.png'));

% 设定多个点 (点1, 2, 3, 4)
waypoints = [
    101,101;

    85,6;  
    96,62;
    92,41; 
    87,23;
    92, 79;  

    59, 11;  
    59,30;
    58,52;
    62,68;
    56,88;

    31,11;
    36,29;
    35,46;
    37,66;
    37,85;

    9,12;
    13,35;
    14,72;
    17,90;
    17,51;

];


num_points = size(waypoints,1);
heights = arrayfun(@(i) HeightData(waypoints(i,2), waypoints(i,1)) , 1:num_points)';
points = [waypoints, heights];

%不使用历史信息进行规划
tic;
[paths, costs, len] = TSP_MultiAstar_no_history(HeightData, waypoints);
toc;

%使用MSD-GWO进行调度优化
disp(num2str(costs))
tic
[Alpha_pos, Alpha_score] = MSD_GWO(costs, num_points, 100, 200);
toc
total_e = 0;
total_l = 0;

allpath = [];   % 存放完整路径

tic
for k = 1:length(Alpha_pos) - 1
    disp("起点：" + Alpha_pos(k) + " 终点：" + Alpha_pos(k+1))
    % 调用A*能量最优路径规划函数
    path = paths{Alpha_pos(k),Alpha_pos(k+1)};

    total_e = total_e + costs(Alpha_pos(k),Alpha_pos(k+1));
    total_l = total_l + len(Alpha_pos(k),Alpha_pos(k+1));
    % 拼接路径（避免重复）
    if k == 1
        allpath = path;
    else
        allpath = [allpath; path(2:end,:)]; % 去掉重复的首点
    end
end

startx = waypoints(Alpha_pos(length(Alpha_pos)),1);
starty = waypoints(Alpha_pos(length(Alpha_pos)),2);
endx   = waypoints(Alpha_pos(1),1);
endy   = waypoints(Alpha_pos(1),2);

path = paths{Alpha_pos(length(Alpha_pos)),1};
total_e = total_e + costs(Alpha_pos(length(Alpha_pos)),1);
total_l = total_l + len(Alpha_pos(length(Alpha_pos)),1);

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
plot3(a(:,1)', a(:,2)', high + 5, 'r-', 'LineWidth', 2)

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

