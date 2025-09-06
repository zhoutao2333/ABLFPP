function path = astar_energypath_final(HeightData,Obstacle_map, startx, starty, endx, endy)
    %% Theta* 搜索算法（支持非均匀代价地图）
    %  HeightData - 地形高度矩阵（值越大表示代价越高）
    %  startx, starty - 起点坐标
    %  endx, endy - 终点坐标

    % ========== 参数设置 ==========
    % 机器人物理参数
    m = 22;       % 质量 (kg)
    v = 0.35;     % 速度 (m/s)
    Pmax = 72;    % 最大功率 (W)
    u = 0.1;      % 摩擦系数
    us = 1.0;     % 静摩擦系数
    g = 9.81;     % 重力加速度
    num_cu = 0;

    % 计算坡度限制
    thetaf = calculateTHm(m, v, Pmax, u);  % 动力限制坡度
    thetaS = atan(us - u);                 % 静摩擦坡度
    thetaM = min(thetaf, thetaS);          % 最大可行坡度
    thetaB = -atan(u);                     % 刹车坡度（下坡）
    % ========== 初始化 ==========
    % 8邻域移动方向
    directions = [0 1; 1 0; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1];
    
    % open_list格式: [x, y, f, g, h]
    open_list = [startx, starty, 0, 0, heuristic(startx, starty, endx, endy, HeightData, m, u, thetaM, thetaB),startx, starty];
    closed_list = [];
    parent_map = containers.Map(); % 存储父节点关系
    
    % ========== 主循环 ==========
    while ~isempty(open_list)
        % 选取f值最小的节点
        [~, idx] = min(open_list(:, 3));
        current = open_list(idx, :);
        open_list(idx, :) = [];
        closed_list = [closed_list; current(1:2)];
        num_cu = num_cu + 1;
        % 检查是否到达终点
        if current(1) == endx && current(2) == endy
            path = reconstruct_path(parent_map, startx, starty, endx, endy);
            path_length = 0;
            for i = 2:size(path,1)
                dx = path(i,1) - path(i-1,1);
                dy = path(i,2) - path(i-1,2);
                dz = HeightData(path(i,2), path(i,1)) - HeightData(path(i-1,2), path(i-1,1));
                path_length = path_length + sqrt(dx^2 + dy^2 + dz^2);
            end
            disp("length :" + path_length)
            disp("energy :" + current(3))
            disp("num" + num_cu)
            return;
        end
 
        % 遍历所有邻居
        for i = 1:size(directions, 1)
            new_x = current(1) + directions(i, 1);
            new_y = current(2) + directions(i, 2);
            
            % 检查越界
            if new_x < 1 || new_x > size(HeightData, 2) || new_y < 1 || new_y > size(HeightData, 1)
                continue;
            end

            if Obstacle_map(new_x,new_y) == 1
                continue;
            end            
            
            % 检查是否已访问
            if ismember([new_x, new_y], closed_list, 'rows')
                continue;
            end
            
            % ===== Theta* 核心优化 =====
            % 计算两种可能的连接方式
            % 方式1：通过当前节点连接
            g1 = current(4) + costx(current(1), current(2), new_x, new_y, HeightData, m, u, thetaM, thetaB);
            % 方式2：通过父节点连接（如果存在父节点）

            if isKey(parent_map, sprintf('%d_%d', current(1), current(2)))
                parent = parent_map(sprintf('%d_%d', current(1), current(2)));
                [hasLOS, g2] = line_of_sight(Obstacle_map,parent(1), parent(2), new_x, new_y, HeightData, m, u, thetaM, thetaB);
                g2 = g2 + parent(3);
                if hasLOS && g2 <= g1
                    % 选择父节点连接
                    h_new = heuristic(new_x, new_y, endx, endy, HeightData, m, u, thetaM, thetaB);
                    f_new = g2 + h_new;

                    % 检查是否在open_list中
                    existing_idx = find(open_list(:,1) == new_x & open_list(:,2) == new_y);
                    if isempty(existing_idx) || all(g2 < open_list(existing_idx, 4))
                        open_list = [open_list; new_x, new_y, f_new, g2, h_new,parent(1), parent(2)];
                        parent_map(sprintf('%d_%d', new_x, new_y)) = [parent(1), parent(2), parent(3)];
                    end
                    continue; % 跳过标准连接
                end
            end
            % ===== 标准A*连接 =====
            h_new = heuristic(new_x, new_y, endx, endy, HeightData, m, u, thetaM, thetaB);
            f_new = g1 + h_new;
            
            % 检查是否在open_list中
            existing_idx = find(open_list(:,1) == new_x & open_list(:,2) == new_y);
            if isempty(existing_idx) || all(g1 < open_list(existing_idx, 4))
                open_list = [open_list; new_x, new_y, f_new, g1, h_new,current(1), current(2)];
                parent_map(sprintf('%d_%d', new_x, new_y)) = [current(1), current(2),current(4)];
            end
        end
    end
    
    % 无可行路径
    path = [];
    disp('未找到可行路径！');
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

function h = heuristic(x1, y1, x2, y2, HeightData,m,u,thetaM, thetaB)
    % 启发式函数，使用欧几里得距离
    dist_xy = sqrt((x2  - x1 )^2 + (y2 - y1 )^2);
    dist_h = HeightData(y2, x2)  - HeightData(y1, x1);
    xxxxx = atan(dist_h / dist_xy);
    if xxxxx > thetaM
        h = m * 9.81 * (dist_h/sin(thetaM)) * (u * cos(xxxxx) + sin(xxxxx));
    elseif xxxxx > thetaB
        h = m * 9.81 * (sqrt((x2 - x1)^2 + (y2 - y1)^2 +  (HeightData(y2, x2) - HeightData(y1, x1))^2)) * (u * cos(xxxxx) + sin(xxxxx));
    else
        h = 0;
    end

end

function path = reconstruct_path(parent_map, starty, starth, endy, endh)
    % 回溯生成路径
    path = [endy, endh];
    key = sprintf('%d_%d', endy, endh);
    while isKey(parent_map, key)
        prev = parent_map(key);
        path = [prev(1:2); path];
        key = sprintf('%d_%d', prev(1), prev(2));
    end
end


function thetam = calculateTHm(m,v,Pmax,u)
    Fmax = Pmax / v;
    temp = asin(Fmax / (m * 9.81 * sqrt(u*u + 1)));
    thetam = temp - atan(u);
end
