clc
clear

%% ���ݳ�ʼ��
height = double(imread('heightmap (8).png'));


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

% ���10���µķǱ߽��
new_points = zeros(10, 2);
for i = 1:10
    % ����2-114��Χ�ڵ�������꣨����߽磩
    x = randi([2, 114]);
    y = randi([2, 114]);
    new_points(i, :) = [x, y];
end

% ���µ���ӵ�ԭ�е㼯����
waypoints = [waypoints; new_points];

num_points = size(waypoints,1);
heights = arrayfun(@(i) height(waypoints(i,2), waypoints(i,1)) , 1:num_points)';
points = [waypoints, heights];



%% ����·��
figure(1)
x=1:115;
y=1:115;
[x1,y1]=meshgrid(x,y);
surf(x1, y1, height), shading interp, colorbar
hold on



% ��ǵ�
for i = 1:num_points
    %text(tsp_points(i,1), tsp_points(i,2), points(i,3), num2str(i), 'Color', 'black');
    plot3(waypoints(i,1) , waypoints(i,2) , points(i,3)+5, 'o',...
                       'MarkerEdgeColor','none',...
                       'MarkerFaceColor','cyan',...
                       'MarkerSize',10);
end




title('TaskPoints 20');
hold off;

