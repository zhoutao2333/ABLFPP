function [path, costA, path_length] = Astar_D(HeightData, waypoints)
    N = size(HeightData,1);
    g = inf(N); 
    parent = zeros(N,N,2);
    Closed = false(N); 
    Open = [];

    m = 22;       % 质量 (kg)
    v = 0.35;     % 速度 (m/s)
    Pmax = 72;    % 最大功率 (W)
    u = 0.1;      % 摩擦系数
    us = 1.0;     % 静摩擦系数
    num_cu = 0;
    start = waypoints(1,:);
    goal = waypoints(2,:);
    % 计算坡度限制
    thetaf = calculateTHm(m, v, Pmax, u);  % 动力限制坡度
    thetaS = atan(us - u);                 % 静摩擦坡度
    thetaM = min(thetaf, thetaS);          % 最大可行坡度
    thetaB = -atan(u);                     % 刹车坡度（下坡）
    Closed = false(size(HeightData));
    Open = [start heuristic(start(1),start(2),goal(1),goal(2),HeightData)];
    g(start(1),start(2)) = 0;
    while ~isempty(Open)
        [~,idx] = min(Open(:,3));
        cur = Open(idx,1:2);
        Open(idx,:) = [];
        num_cu = num_cu + 1;    
        Closed(cur(1),cur(2)) = true;
        if isequal(cur,goal)
            disp("num" + num_cu);
            path = ReconstructPath(parent,start,goal);
            path_length = g(goal(1), goal(2));
            costA = 0;
            for i = 2:size(path, 1)
                x1 = path(i-1, 1);
                y1 = path(i-1, 2);
                x2 = path(i, 1);
                y2 = path(i, 2);
                energy = energy_cost(x1,y1,x2,y2,HeightData,m,u,thetaM,thetaB);  % 单段能耗
                costA = costA + energy;
            end
            return
        end

        for d=[0 1; 1 0; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1]'
            nb = cur+d';
            if nb(1)<1 || nb(1)>size(HeightData,1)||nb(2) < 1 || nb(2)>size(HeightData,2), continue; end
            if Closed(nb(1),nb(2)), continue; end
            ng = g(cur(1),cur(2))+cost(cur(1),cur(2),nb(1),nb(2),HeightData);
            if ng < g(nb(1),nb(2))
                g(nb(1),nb(2)) = ng;
                parent(nb(1),nb(2),1:2) = cur;
                parent(nb(1),nb(2),3) = g(cur(1),cur(2));
                f = ng + heuristic(nb(1),nb(2),goal(1),goal(2),HeightData);
                Open = [Open; nb f];
            end
        end
    end

    path = [];
    costA = 0;
    path_length = 0;
end


function c = cost(x1, y1, x2, y2, HeightData)
    c = sqrt((x2  - x1 )^2 + (y2 - y1)^2 +  (HeightData(y2, x2)  - HeightData(y1, x1))^2);
end

function h = heuristic(x1, y1, x2, y2, HeightData)
    h = sqrt((x2  - x1 )^2 + (y2 - y1)^2 +  (HeightData(y2, x2)  - HeightData(y1, x1))^2);
end

function c = energy_cost(x1, y1, x2, y2, HeightData, m,u, thetaM , thetaB)
    % 计算移动成本，考虑地形高度
    dist_xy = sqrt((x2  - x1 )^2 + (y2 - y1)^2);
    dist_h = abs(HeightData(y2, x2)  - HeightData(y1, x1));
    xxxxx = atan(dist_h / dist_xy);
    if xxxxx > thetaM
        c = Inf;
    elseif xxxxx > thetaB
        c = m * 9.81 * (sqrt((x2 - x1)^2 + (y2 - y1)^2 +  (HeightData(y2, x2) - HeightData(y1, x1))^2)) * (u * cosd(xxxxx) + sind(xxxxx));
    else
        c = 0;
    end

 end


function thetam = calculateTHm(m,v,Pmax,u)
    Fmax = Pmax / v;
    temp = asin(Fmax / (m * 9.81 * sqrt(u*u + 1)));
    thetam = temp - atan(u);
end

function path = ReconstructPath(parent,start,goal)
    path = goal;
    cur = goal;
    while ~isequal(cur,start)
        p = squeeze(parent(cur(1),cur(2),1:2))';
        if all(p==0), break; end
        path = [p; path];
        cur = p;
    end
end


