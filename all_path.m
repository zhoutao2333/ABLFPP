HeightData = double(imread('heightmap (8).png'));


% �趨����� (��1, 2, 3, 4)
waypoints = [
    90,91;
    3, 4;  % ��1
    54, 87; % ��2
    14, 84;  % ��3
    77, 27;   % ��4
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
heights = arrayfun(@(i) HeightData(waypoints(i,2), waypoints(i,1)), 1:num_points)';
points = [waypoints, heights];

% �������е�� (i, j) ֮�������·��
tic;
[paths, costs] = IAstar_History(HeightData, waypoints);
toc;
disp("costs is")
disp(costs);
disp("end")

disp("paht start")
disp(paths)
disp("path end")

%% ����·��
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
    %text(tsp_points(i,1), tsp_points(i,2), points(i,3), num2str(i), 'Color', 'black');
    plot3(waypoints(i,1), waypoints(i,2), points(i,3)+5, 'o',...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','cyan',...
                       'MarkerSize',10);
end

title('TaskPoints 20');
hold off;

