clc
clear

HeightData = double(imread('heightmap (8).png'));
Obstacle_map = HeightData;
Obstacle_map(:) = 0;

disp(HeightData)
disp(size(HeightData))
% ����յ������ 

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