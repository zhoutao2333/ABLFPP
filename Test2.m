%TEST2

% 下载数据
load  peaks.mat HeightData

% 起点终点网格点 
waypoints = [
    51,10
    53,84
];

% 最佳路径搜索
tStart = tic;   % 开始计时
%选择路径规划算法D_D,D_E,Astar_D,Astar_E,Zstar,Improved_Astar
bestpath = Improved_Astar(HeightData, waypoints);

elapsedTime = toc(tStart);  % 获取耗时
fprintf('总耗时：%.4f 秒\n', elapsedTime);
%% 绘制路径
for i=1:size(bestpath,1)
    a(i,1)=bestpath(i,1);
    a(i,2)=bestpath(i,2);
end
disp("size" + size(bestpath,1));
figure(1)
x=1:100;
y=1:100;
[x1,y1]=meshgrid(x,y);



surf( x1,  y1, HeightData), shading interp, colorbar

hold on
plot3(a(1,1)', a(1,2)',HeightData(a(1,2),a(1,1)) + 5,'rp','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','c',...
                       'MarkerSize',10)
plot3( a(end,1)',a(end,2)',HeightData(a(end,2),a(end,1)) + 5,'bo','MarkerSize',2,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','b',...
                       'MarkerSize',10)
%text(10 * a(1,1)',10 * a(1,2)',HeightData(a(1,2),a(1,1)) + 10,'S');
%text(10 * a(end,1)',10 * a(end,2)',HeightData(a(end,2),a(end,1)) + 10,'T');

%title('Ours','fontsize',12)
set(gcf, 'Renderer', 'ZBuffer')
hold on
for i=1:size(a,1)
    height(i) = HeightData(a(i,2),a(i,1));
end
plot3( a(:,1)',a(:,2)',height+2,'-','LineWidth',3)
