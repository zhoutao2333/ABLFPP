height= imread('heightmap (8).png');

xk = 51;yk = 10;
x2 = 53;y2 = 89;
figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
disp(height(51,10))
disp(height(53,89))
disp(height(y2,x2))
disp(height(yk,xk))
disp("heheh" + (height(53,89)-height(51,10)))
disp("he" + (height(x2,y2) - height(xk,yk)))

disp(class(height(51,10)))
surf( x1,  y1, height), shading interp, colorbar
hold on


title('最佳个体适应度变化趋势')
xlabel('x')
ylabel('y')