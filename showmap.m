clc
clear

HeightData = double(imread('heightmap.png'));
Obstacle_map = HeightData;
Obstacle_map(:) = 0;

disp(HeightData)
disp(size(HeightData))
% 起点终点网格点 

figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
disp(class(HeightData))
surf(x1, y1, HeightData), shading interp, colorbar
hold on





title('Map Build')
xlabel('x')
ylabel('y')