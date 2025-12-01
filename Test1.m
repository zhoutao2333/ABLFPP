clc
clear

% 数据初始化
HeightData = double(imread('heightmap.png'));

waypoints = [
    94,89;
    19, 15;  
];


tic;
[paths, costs, len] = D_E(HeightData, waypoints);
toc;
disp(num2str(costs))
disp(len)


for i=1:size(paths,1)
    a(i,1)=paths(i,1);
    a(i,2)=paths(i,2);
end
disp("size" + size(paths,1));
figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);



surf(x1, y1, HeightData), shading interp, colorbar

hold on
plot3(a(1,1)', a(1,2)',HeightData(a(1,2),a(1,1)) + 10,'rp','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','c',...
                       'MarkerSize',10)
plot3(a(end,1)',a(end,2)',HeightData(a(end,2),a(end,1)) + 10,'bo','MarkerSize',12,...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','b',...
                       'MarkerSize',10)
%text(a(1,1)',a(1,2)',HeightData(a(1,2),a(1,1)) + 10,'S');
%text(a(end,1)',a(end,2)',HeightData(a(end,2),a(end,1)) + 10,'T');
xlabel('m','fontsize',12);
ylabel('m','fontsize',12);
zlabel('m','fontsize',12);
%title('Ours','fontsize',12)
set(gcf, 'Renderer', 'ZBuffer')
hold on
for i=1:size(a,1)
    high(i) = HeightData(a(i,2),a(i,1));
end
plot3(a(:,1)',a(:,2)',high + 10,'-','LineWidth',3)
