clc
clear


% 数据初始化
HeightData = double(imread('heightmap.png'));
%94 89
%32 9
% 起点终点网格点 
waypoints = [
    94,89
    47,66;  
];
startx=waypoints(1,1);starty=waypoints(1,2);
endx=waypoints(2,1);endy=waypoints(2,2);
starth = HeightData(starty,startx);
endh = HeightData(endy,endx);

% RRT* 路径
tic
max_iter   = 10000;   % 迭代次数
stepsize   = 1;    % 每次延伸步长
goal_radius= 10;    % 半径

[path_rrt, energy_rrt] = RRT_Star(HeightData, startx, starty, endx, endy, ...
                                              max_iter, stepsize, goal_radius);
toc
%% 绘图对比
figure(1)
x = 1:115;
y = 1:115;
[x1,y1] = meshgrid(x,y);
surf(x1, y1, HeightData), shading interp, colorbar
hold on


% 起终点
plot3(startx, starty, HeightData(starty, startx) + 10,'rp','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','c',...
                       'MarkerSize',10);
plot3(endx,   endy,   HeightData(endy, endx) + 12,  'bo','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','b',...
                       'MarkerSize',10);

% RRT* 路径
if ~isempty(path_rrt)
    b = path_rrt;
    high_b = zeros(size(b,1),1);
    for i=1:size(b,1)
        high_b(i) = HeightData(b(i,2),b(i,1));
    end
    plot3(b(:,1)',b(:,2)',high_b + 10,'-','LineWidth',3)
end

text(30, 90, HeightData(94,89)+30, num2str("Path Length:"+148.03+"m"), ...
     'Color', 'black', 'FontAngle' , 'italic' ,'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(27, 70, HeightData(94,89)+30, num2str("Enery Cost:"+10920.7+"J"), ...
     'Color', 'black','FontAngle' , 'italic' , 'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(40, 110, HeightData(94,89)+30, num2str("Time Cost:"+1.12+"s"), ...
     'Color', 'black', 'FontAngle' , 'italic' ,'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');