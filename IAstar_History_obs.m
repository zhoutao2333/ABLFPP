function [paths, costs,len] = IAstar_History_obs(HeightData, Obstacle_map, waypoints)
    % 使用 IA*_H 计算所有点对路径
    num_points = size(waypoints, 1);
    paths = cell(num_points, num_points);
    costs = zeros(num_points, num_points);
    len = zeros(num_points, num_points);
    N = size(HeightData,1);

    g = inf(N); 
    parent = zeros(N,N,3);
    Closed = false(N); 
    Open = [];

    m = 22;       % 质量 (kg)
    v = 0.35;     % 速度 (m/s)
    Pmax = 72;    % 最大功率 (W)
    u = 0.1;      % 摩擦系数
    us = 1.0;     % 静摩擦系数


    % 计算坡度限制
    thetaf = calculateTHm(m, v, Pmax, u);  % 动力限制坡度
    thetaS = atan(us - u);                 % 静摩擦坡度
    thetaM = min(thetaf, thetaS);          % 最大可行坡度
    thetaB = -atan(u);                     % 刹车坡度（下坡）
    FLAG = true;
    % ===== 遍历所有点对 =====
    for i = 1:num_points
        FLAG = false;
        for j = 1:num_points
            if i == j
                continue
            end
            start = waypoints(i,:);
            goal  = waypoints(j,:);
            if FLAG == false
                % 第一次：运行完整 A*
                [path, Open, Closed, g, parent,costA,path_length] = Astar(HeightData, Obstacle_map, start, goal, g, parent, m, u, thetaM, thetaB);
                FLAG = true;
            else
                % 后续：直接调用 IA_H 增量更新
                [path, Open, Closed, g, parent,costA,path_length] = Astar_HIS(HeightData, Obstacle_map, start, goal, Open, Closed, g, parent, m, u, thetaM, thetaB);
            end
            % 保存结果
            paths{i,j} = path;

            if isempty(path)
                costs(i,j) = inf;
            else
                costs(i,j) = costA;
                len(i,j) = path_length;
            end
        end
    end
end

function [path, Open, Closed, g, parent,costA,path_length] = Astar_HIS(HeightData, Obstacle_map, start, goal, Open, Closed, g, parent, m, u, thetaM, thetaB)
    if Closed(goal(1),goal(2))
        path = ReconstructPath(parent, start, goal);
        costA = g(goal(1),goal(2));
        path_length = 0;
        for i = 2:size(path,1)
                dx = path(i,1) - path(i-1,1);
                dy = path(i,2) - path(i-1,2);
                dz = HeightData(path(i,2), path(i,1)) - HeightData(path(i-1,2), path(i-1,1));
                path_length = path_length + sqrt(dx^2 + dy^2 + dz^2);
        end
        return;
    end

    Open = UpdateOpenHeuristic(Open, goal,g,HeightData, m, u, thetaM, thetaB);
    [path, Open, Closed, g, parent,costA,path_length] = Astar_continue(HeightData,Obstacle_map, start, goal, Open, Closed, g, parent, m, u, thetaM, thetaB);
end 


function [path, Open, Closed, g, parent,costA,path_length] = Astar(HeightData, Obstacle_map, start, goal, g, parent, m, u, thetaM, thetaB)
    Closed = false(size(HeightData));
    Open = [start heuristic(start(1),start(2),goal(1),goal(2),HeightData, m, u, thetaM, thetaB)];
    g(:) = inf;
    g(start(1),start(2)) = 0;
    while ~isempty(Open)
        [~,idx] = min(Open(:,3));
        cur = Open(idx,1:2);
        Open(idx,:) = [];  
        Closed(cur(1),cur(2)) = true;
        if isequal(cur,goal)
            path = ReconstructPath(parent,start,goal);
            costA = g(goal(1), goal(2));
            path_length = 0;
            for i = 2:size(path,1)
                    dx = path(i,1) - path(i-1,1);
                    dy = path(i,2) - path(i-1,2);
                    dz = HeightData(path(i,2), path(i,1)) - HeightData(path(i-1,2), path(i-1,1));
                    path_length = path_length + sqrt(dx^2 + dy^2 + dz^2);
            end
            return
        end

        for d=[0 1; 1 0; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1]'
            nb = cur+d';
            if nb(1)<1 || nb(1)>size(HeightData,1)||nb(2) < 1 || nb(2)>size(HeightData,2), continue; end
            if Closed(nb(1),nb(2)), continue; end
            if Obstacle_map(nb(1),nb(2)), continue; end
            ng = g(cur(1),cur(2))+cost(cur(1),cur(2),nb(1),nb(2),HeightData, m, u, thetaM, thetaB);
            %使用ia* 
           if ~isequal(cur, start) && any(parent(cur(1), cur(2), :))
                parent_node = squeeze(parent(cur(1), cur(2), :))';
                [hasLOS, g2] = line_of_sight(Obstacle_map, parent_node(1), parent_node(2), nb(1), nb(2), HeightData, m, u, thetaM, thetaB);
                g2 = g2 + parent_node(3);

                if hasLOS && g2 <= ng
                    % 选择父节点连接
                    if g2 < g(nb(1), nb(2))
                        g(nb(1), nb(2)) = g2;
                        parent(nb(1), nb(2), 1:2) = parent_node(1:2);
                        parent(nb(1), nb(2), 3) = parent_node(3);

                        h_new = heuristic(nb(1), nb(2), goal(1),goal(2), HeightData, m, u, thetaM, thetaB);
                        f_new = g2 + h_new;

                        % 添加到开放列表或更新
                        Open = [Open; nb, f_new];
                    end
                    continue; % 跳过标准连接
                end
            end
            if ng < g(nb(1),nb(2))
                g(nb(1),nb(2)) = ng;
                parent(nb(1),nb(2),1:2) = cur;
                parent(nb(1),nb(2),3) = g(cur(1),cur(2));
                f = ng+heuristic(nb(1),nb(2),goal(1),goal(2),HeightData, m, u, thetaM, thetaB);
                Open = [Open; nb f];
            end
        end
    end

    path = [];
end

function [path, Open, Closed, g, parent,costA,path_length] = Astar_continue(HeightData,Obstacle_map, start, goal, Open, Closed, g, parent, m, u, thetaM, thetaB)
    %继续 A*
    costA = 0;
    while ~isempty(Open)
        [~,idx] = min(Open(:,3));
        cur = Open(idx,1:2);

        Open(idx,:) = [];
        if isequal(cur,goal)
            path = ReconstructPath(parent,start,goal);
            costA = g(goal(1), goal(2));
            path_length = 0;
            for i = 2:size(path,1)
                    dx = path(i,1) - path(i-1,1);
                    dy = path(i,2) - path(i-1,2);
                    dz = HeightData(path(i,2), path(i,1)) - HeightData(path(i-1,2), path(i-1,1));
                    path_length = path_length + sqrt(dx^2 + dy^2 + dz^2);
            end
            return
        end
        Closed(cur(1),cur(2)) = true;
        for d=[0 1; 1 0; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1]'
            nb = cur+d';
            if nb(1)<1 || nb(1)>size(HeightData,1)||nb(2) < 1 || nb(2)>size(HeightData,2), continue; end
            %if HeightData(nb(1),nb(2))==0, continue; end
            if Closed(nb(1),nb(2)), continue; end
            if Obstacle_map(nb(1),nb(2)), continue; end
            ng = g(cur(1),cur(2))+cost(cur(1),cur(2),nb(1),nb(2),HeightData, m, u, thetaM, thetaB);
           if ~isequal(cur, start) && any(parent(cur(1), cur(2), :))
                parent_node = squeeze(parent(cur(1), cur(2), :))';
                [hasLOS, g2] = line_of_sight(Obstacle_map, parent_node(1), parent_node(2), nb(1), nb(2), HeightData, m, u, thetaM, thetaB);
                g2 = g2 + parent_node(3);
                if hasLOS && g2 <= ng
                    % 选择父节点连接
                    if g2 < g(nb(1), nb(2))
                        g(nb(1), nb(2)) = g2;
                        parent(nb(1), nb(2), 1:2) = parent_node(1:2);
                        parent(nb(1), nb(2), 3) = parent_node(3);

                        h_new = heuristic(nb(1), nb(2), goal(1),goal(2), HeightData, m, u, thetaM, thetaB);
                        f_new = g2 + h_new;

                        % 添加到开放列表或更新
                        Open = [Open; nb, f_new];
                    end
                    continue; % 跳过标准连接
                end
            end
            if ng < g(nb(1),nb(2))
                g(nb(1),nb(2)) = ng;
                parent(nb(1),nb(2),1:2) = cur;
                parent(nb(1),nb(2),3) = g(cur(1),cur(2));
                f = ng+heuristic(nb(1),nb(2),goal(1),goal(2),HeightData, m, u, thetaM, thetaB);
                Open = [Open; nb f];
            end
        end
    end
    path = [];
end


function Open = UpdateOpenHeuristic(Open, goal,g,HeightData, m, u, thetaM, thetaB)
    if isempty(Open), return; end
    for i = 1:size(Open,1)
        node = Open(i,1:2);
        h_new = heuristic(node(1),node(2), goal(1),goal(2),HeightData, m, u, thetaM, thetaB); % 新目标下的启发式
        Open(i,3) = g(node(1),node(2)) + h_new;     % 更新 f = g+h
    end
end


function c = cost(x1, y1, x2, y2, HeightData, m,u, thetaM , thetaB)
    % 计算移动成本，考虑地形高度

    dist_xy = sqrt((x2  - x1 )^2 + (y2 - y1)^2);
    dist_h = HeightData(y2, x2)  - HeightData(y1, x1);
    xxxxx = atan(dist_h / dist_xy);
    if xxxxx > thetaM
        c = Inf;
    elseif xxxxx > thetaB
        c = m * 9.81 * (sqrt((x2 - x1)^2 + (y2 - y1)^2 +  (HeightData(y2, x2) - HeightData(y1, x1))^2)) * (u * cos(xxxxx) + sin(xxxxx));
    else
        c = 0;
    end

 end

function h = heuristic(x1, y1, x2, y2, HeightData,m,u,thetaM, thetaB)
    % 启发式函数，使用欧几里得距离
    dist_xy = sqrt((x2  - x1 )^2 + (y2 - y1 )^2);
    dist_h = HeightData(y2, x2)  - HeightData(y1, x1);
    xxxxx = atan(dist_h / dist_xy);
    if xxxxx > thetaM
        h = m * 9.81 * (dist_h/sin(thetaM)) * (u * cos(thetaM) + sin(thetaM));
    elseif xxxxx > thetaB
        h = m * 9.81 * (sqrt((x2 - x1)^2 + (y2 - y1)^2 +  (HeightData(y2, x2) - HeightData(y1, x1))^2)) * (u * cos(xxxxx) + sin(xxxxx));
    else
        h = 0;
    end

end



function new = random_move(pos,N,map)
    new = pos + randi([-1 1],1,2);
    new = max(min(new,N),1);
    while map(new(1),new(2))==0
        new = pos + randi([-1 1],1,2);
        new = max(min(new,N),1);
    end
end

function cost = compute_path_cost(path, HeightData)
    cost = 0;
    for k=2:size(path,1)
        y1=path(k-1,1); x1=path(k-1,2);
        y2=path(k,1);   x2=path(k,2);
        cost = cost + sqrt((x2-x1)^2+(y2-y1)^2+(HeightData(y2,x2)-HeightData(y1,x1))^2);
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

function [hasLOS, totalCost] = line_of_sight(Obstacle_map,x1, y1, x2, y2, HeightData, m, u, thetaM, thetaB)
    % 实现完整的通视检测和代价计算（包含水平和垂直方向处理）
    totalCost = 0;
    % 初始化参数
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    s_x = sign(x2 - x1);
    s_y = sign(y2 - y1);
    tx = 2 * dx;
    ty = 2 * dy;
    % 检查起点和终点是否相同
    if dx == 0 && dy == 0
        hasLOS = true;
        return;
    end
    g_x = x2;
    g_y = y2;
    % 选择主方向（水平或垂直）
    if tx > ty
        % ===== 水平方向为主轴 =====
        tau = (ty - tx)/2;
        rho = (ty + tx)/2;
        x = x1;
        y = y1;
        e = 0;
        x_temp = x1;
        y_temp = y1;
        % 检查起点是否可行
        cell_cost = costx(x, y, x+s_x, y, HeightData, m, u, thetaM, thetaB);
        totalCost = totalCost + 0.5 * cell_cost;
        if isinf(cell_cost)
            hasLOS = false;
            totalCost = Inf;
            return;
        end
        while x ~= x2
            if e > tau
                % x方向移动
                x = x + s_x;
                if Obstacle_map(x,y) == 1
                    hasLOS = false;
                    return
                end      
                e = e - ty;
                cost = costx(x_temp, y_temp, x, y, HeightData, m, u, thetaM, thetaB);
                x_temp = x;
                if isinf(cost)
                    hasLOS = false;
                    totalCost = Inf;
                    return;
                end
                if rho + e < ty
                    w = (rho + e)/ty;
                    totalCost = totalCost + w * cost;
                else
                    totalCost = totalCost + cost;
                end
            elseif e < tau
                % y方向移动
                y = y + s_y;
                e = e + tx;
                if Obstacle_map(x,y) == 1
                    hasLOS = false;
                    return
                end     
                cost = costx(x_temp, y_temp, x, y, HeightData, m, u, thetaM, thetaB);
                y_temp = y;
                if isinf(cost)
                    hasLOS = false;
                    totalCost = Inf;
                    return;
                end
                    totalCost = totalCost +  (1-w) * cost;
            else
                % 对角线移动
                x = x + s_x;
                y = y + s_y;
                if Obstacle_map(x,y) == 1
                    hasLOS = false;
                    return
                end     
                e = e + tx - ty;
                cost = costx(x_temp, y_temp, x, y, HeightData, m, u, thetaM, thetaB);
                x_temp = x;
                y_temp = y;
                if isinf(cost)
                    hasLOS = false;
                    totalCost = Inf;
                    return;
                end
                totalCost = totalCost + cost;
            end
        end
        
        % 终点代价调整
        totalCost = totalCost - 0.5 * costx(x2-s_x, y2, x2, y2, HeightData, m, u, thetaM, thetaB);
        if dx ~= 0
            totalCost = totalCost * sqrt(tx^2 + ty^2)/tx;
        end
    elseif tx < ty
        % ===== 垂直方向为主轴 =====
        tau = (tx - ty)/2;
        rho = (tx + ty)/2;
        x = x1;
        y = y1;
        e = 0;
        x_temp = x1;
        y_temp = y1;
        % 检查起点是否可行
        cell_cost = costx(x, y, x, y + s_y, HeightData, m, u, thetaM, thetaB);
        if isinf(cell_cost)
            hasLOS = false;
            totalCost = Inf;
            return;
        end
        totalCost = totalCost + 0.5 * cell_cost;
        while y ~= y2
            if e > tau
                % y方向移动
                y = y + s_y;
                if Obstacle_map(x,y) == 1
                    hasLOS = false;
                    return
                end     
                e = e - tx;
                cost = costx(x_temp, y_temp, x, y, HeightData, m, u, thetaM, thetaB);
                y_temp = y;
                if isinf(cost)
                    hasLOS = false;
                    totalCost = Inf;
                    return;
                end
                if rho + e < tx
                    w = (rho + e)/tx;
                    totalCost = totalCost + w * cost;
                else
                    totalCost = totalCost + cost;
                end
            elseif e < tau
                % x方向移动
                x = x + s_x;
                e = e + ty;
                if Obstacle_map(x,y) == 1
                    hasLOS = false;
                    return
                end     
                cost = costx(x_temp, y_temp, x, y, HeightData, m, u, thetaM, thetaB);
                x_temp = x;
                if isinf(cost)
                    hasLOS = false;
                    totalCost = Inf;
                    return;
                end
                totalCost = totalCost + (1-w) * cost;
            else
                % 对角线移动
                x = x + s_x;
                y = y + s_y;
                if Obstacle_map(x,y) == 1
                    hasLOS = false;
                    return
                end     
                e = e + ty - tx;
                cost = costx(x_temp, y_temp, x, y, HeightData, m, u, thetaM, thetaB);
                x_temp = x;
                y_temp = y;
                if isinf(cost)
                    hasLOS = false;
                    totalCost = Inf;
                    return;
                end
                totalCost = totalCost + cost;
            end
        end
        
        % 终点代价调整
        totalCost = totalCost - 0.5 * costx(x2, y2-s_y, x2, y2, HeightData, m, u, thetaM, thetaB);
        if dy ~= 0
            totalCost = totalCost * sqrt(tx^2 + ty^2)/ty;
        end
    else
        hasLOS = false;
        return;
    end
    hasLOS = true;
end

function cell_cost = get_cell_cost(x, y, x2, y2, HeightData, m, u, thetaM, thetaB)
    % 计算单个格子的代价
    if x == x2 && y == y2
        cell_cost = 0;
        return;
    end
    % 计算与下一个格子的坡度
    next_x = x + sign(x2 - x);
    next_y = y + sign(y2 - y);
    dist_xy = sqrt((next_x - x)^2 + (next_y - y)^2);
    dist_h = HeightData(next_y, next_x) - HeightData(y, x);
    slope = atan(dist_h / dist_xy);
    
    if slope > thetaM
        cell_cost = Inf;
    elseif slope > thetaB
        cell_cost = m * 9.81 * sqrt(dist_xy^2 + dist_h^2) * (u * cos(slope) + sin(slope));
    else
        cell_cost = 0;
    end
end

function c = costx(x1, y1, x2, y2, HeightData, m,u, thetaM , thetaB)
    % 计算移动成本，考虑地形高度

    dist_xy = sqrt((x2  - x1 )^2 + (y2 - y1)^2);
    dist_h = HeightData(y2, x2)  - HeightData(y1, x1);
    xxxxx = atan(dist_h / dist_xy);
    if xxxxx > thetaM
        c = Inf;
    elseif xxxxx > thetaB
        c = m * 9.81 * (sqrt((x2 - x1)^2 + (y2 - y1)^2 +  (HeightData(y2, x2) - HeightData(y1, x1))^2)) * (u * cos(xxxxx) + sin(xxxxx));
    else
        c = 0;
    end

 end

