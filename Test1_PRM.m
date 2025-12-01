%====TEST1_PRM method=====%
HeightData = double(imread('heightmap.png'));

% 起点终点网格点 
waypoints = [
    94,89
    47,66;    
];
startx=waypoints(1,1);starty=waypoints(1,2);
endx=waypoints(2,1);endy=waypoints(2,2);


% PRM 路径

n_samples    = 2000;   % 采样点数量（不含起终点）
k_neighbors  = 10;     % 每个节点最多连接的近邻数量
max_radius   = 5;   % 邻居最大连接距离（网格单位）
tic
[path_prm, energy_prm] = PRM(HeightData, startx, starty, endx, endy, ...
                                        n_samples, k_neighbors, max_radius);
toc
%% 绘制对比
figure(1)
x = 1:size(HeightData,2);
y = 1:size(HeightData,1);
[x1,y1] = meshgrid(x,y);
surf(x1, y1, HeightData), shading interp, colorbar
hold on

% 起点/终点
plot3(startx, starty, HeightData(starty, startx) + 10,'rp','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','c',...
                       'MarkerSize',10);
plot3(endx,   endy,   HeightData(endy, endx) + 12,  'bo','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','b',...
                       'MarkerSize',10);


if ~isempty(path_prm)
    c = path_prm;
    high_c = zeros(size(c,1),1);
    for i = 1:size(c,1)
        high_c(i) = HeightData(c(i,2), c(i,1));
    end
    plot3(c(:,1)', c(:,2)', high_c + 8, '-', 'LineWidth', 3)
end



text(30, 90, HeightData(94,89)+35, num2str("Path Length:"+235.39+"m"), ...
     'Color', 'black', 'FontAngle' , 'italic' ,'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(27, 70, HeightData(94,89)+35, num2str("Enery Cost:"+12558.64+"J"), ...
     'Color', 'black','FontAngle' , 'italic' , 'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(40, 110, HeightData(94,89)+35, num2str("Time Cost:"+0.43+"s"), ...
     'Color', 'black', 'FontAngle' , 'italic' ,'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');