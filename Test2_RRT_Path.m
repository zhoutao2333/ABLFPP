%====RRT*以路径长度为目标====%
load  peaks.mat HeightData

waypoints = [
    51,10
    53,84
];

startx=waypoints(1,1);starty=waypoints(1,2);
endx=waypoints(2,1);endy=waypoints(2,2);
starth = HeightData(starty,startx);
endh = HeightData(endy,endx);

%% RRT* 路径
tic
max_iter   = 2000;   % 迭代次数
stepsize   = 1;    % 每次延伸步长
goal_radius= 10;    % 半径

[path_rrt, energy_rrt] = RRT_Star_length(HeightData, startx, starty, endx, endy, ...
                                              max_iter, stepsize, goal_radius);
toc
%% 绘图对比
figure(1)
x = 1:100;
y = 1:100;
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
    plot3(b(:,1)',b(:,2)',high_b + 8,'-','LineWidth',3)
end

text(74, 45, HeightData(94,89)+30, num2str("Enery Cost:Inf J"), ...
     'Color', 'black','FontAngle' , 'italic' , 'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(77, 35, HeightData(94,89)+30, num2str("Path Length:"+207.26+"m"), ...
     'Color', 'black', 'FontAngle' , 'italic' ,'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(70, 25, HeightData(94,89)+30, num2str("Time Cost:"+0.28+"s"), ...
     'Color', 'black', 'FontAngle' , 'italic' ,'FontSize', 12, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');