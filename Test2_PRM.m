clc
clear

%% 数据初始化
load  peaks.mat HeightData

% 起点终点网格点 
startx = 51; starty = 10;
endx   = 53; endy   = 84;

%% PRM 路径

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

disp(path_prm)
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



    text(78, 45, HeightData(94,89)+30, num2str("Enery Cost:7924.98 J"), ...
         'Color', 'black','FontAngle' , 'italic' , 'FontSize', 12, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(77, 35, HeightData(94,89)+30, num2str("Path Length:"+263.74+"m"), ...
         'Color', 'black', 'FontAngle' , 'italic' ,'FontSize', 12, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(70, 25, HeightData(94,89)+30, num2str("Time Cost:"+0.45+"s"), ...
         'Color', 'black', 'FontAngle' , 'italic' ,'FontSize', 12, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');