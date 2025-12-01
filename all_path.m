% 显示所有路径
HeightData = double(imread('heightmap.png'));

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
heights = arrayfun(@(i) HeightData(waypoints(i,2), waypoints(i,1)), 1:num_points)';
points = [waypoints, heights];

% 计算所有点对 (i, j) 之间的最优路径
tic;
[paths, costs] = IAstar_History(HeightData, waypoints);
toc;
disp("costs is")
disp(costs);
disp("end")

disp("paht start")
disp(paths)
disp("path end")

%% 绘制路径
figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
surf(x1, y1, HeightData), shading interp, colorbar
hold on


for i = 1:num_points
    for j = i+1:num_points
        path = paths{i, j};
        if ~isempty(path)
            height = arrayfun(@(idx) HeightData(path(idx,2), path(idx,1)), 1:size(path,1));
            plot3(path(:,1), path(:,2), height + 5, '-', 'LineWidth', 1.5, 'Color', 'red');
        end
    end
end

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

