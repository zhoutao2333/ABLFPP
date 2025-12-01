function [path, best_length] = RRT_Star_length(HeightData, startx, starty, endx, endy, max_iter, stepsize_unused, goal_radius)
    % RRT* 版本：在 8 邻接格子上搜索，代价只用 3D 欧氏距离
    % 不考虑坡度限制，所有邻居边（不越界）都视为可行

    [LevelGrid, PortGrid] = size(HeightData);

    % RRT* 初始化
    % 节点结构：x, y, cost(累计距离), parent(索引)
    nodes(1).x      = startx;
    nodes(1).y      = starty;
    nodes(1).cost   = 0;   % 当前从起点到该点的累计 3D 距离
    nodes(1).parent = 0;
    node_count      = 1;

    best_length      = inf;   % 最短路径长度
    best_goal_parent = -1;

    % 采样中对终点的偏置概率（goal bias）
    goal_bias = 0.1;   % 10% 直接采样目标点

    % RRT* 邻域半径系数（可根据地图尺寸适当调）
    gamma_rrt = 20;    % 越大 rewiring 越积极

    for iter = 1:max_iter

        %采样随机点  
        if rand < goal_bias
            x_rand = endx;
            y_rand = endy;
        else
            x_rand = randi(PortGrid);
            y_rand = randi(LevelGrid);
        end

        %找最近节点
        xs = [nodes.x];
        ys = [nodes.y];
        dists = sqrt((xs - x_rand).^2 + (ys - y_rand).^2);
        [~, idx_near] = min(dists);
        x_near = nodes(idx_near).x;
        y_near = nodes(idx_near).y;

        %只走到 8 邻接格子
        dir = [x_rand - x_near, y_rand - y_near];

        if dir(1) == 0 && dir(2) == 0
            % 采样点刚好就是已有节点，跳过
            continue;
        end

        dx = sign(dir(1));   % -1 / 0 / 1
        dy = sign(dir(2));   % -1 / 0 / 1

        % 8 邻接一步
        x_new = x_near + dx;
        y_new = y_near + dy;

        % 越界检查
        if x_new < 1 || x_new > PortGrid || y_new < 1 || y_new > LevelGrid
            continue;
        end

        % 邻接合法性检查（只看 8 邻居 + 越界） 
        if ~edge_valid_rrt_dist(x_near, y_near, x_new, y_new, HeightData)
            continue;
        end

        % 边的距离代价（3D 欧氏距离）
        edge_cost = cost_dist(x_near, y_near, x_new, y_new, HeightData);
        if isinf(edge_cost)
            continue;
        end

        %找邻域节点
        % 邻域半径 r(n) ≈ gamma * (log n / n)^(1/2)
        n = node_count;
        r = gamma_rrt * (log(n + 1) / (n + 1))^(1/2);
        dists_new = sqrt((xs - x_new).^2 + (ys - y_new).^2);
        X_near_idx = find(dists_new <= r);

        %选择最优父节点
        best_parent   = idx_near;
        best_new_cost = nodes(idx_near).cost + edge_cost;

        for k = 1:length(X_near_idx)
            i  = X_near_idx(k);
            x_i = nodes(i).x;
            y_i = nodes(i).y;

            % 只允许 8 邻接
            if abs(x_i - x_new) > 1 || abs(y_i - y_new) > 1
                continue;
            end

            % 边 i -> new 是否可行（不考虑坡度，只检查邻接 & 越界）
            if ~edge_valid_rrt_dist(x_i, y_i, x_new, y_new, HeightData)
                continue;
            end

            edge_cost_i = cost_dist(x_i, y_i, x_new, y_new, HeightData);
            if isinf(edge_cost_i)
                continue;
            end

            new_cost_candidate = nodes(i).cost + edge_cost_i;
            if new_cost_candidate < best_new_cost
                best_new_cost = new_cost_candidate;
                best_parent   = i;
            end
        end

        % 将 x_new 插入树 
        node_count = node_count + 1;
        nodes(node_count).x      = x_new;
        nodes(node_count).y      = y_new;
        nodes(node_count).cost   = best_new_cost;   % 累计 3D 距离
        nodes(node_count).parent = best_parent;

        %Rewire 邻居节点
        for k = 1:length(X_near_idx)
            i = X_near_idx(k);
            if i == best_parent || i == node_count
                continue;
            end

            x_i = nodes(i).x;
            y_i = nodes(i).y;

            % 只允许 8 邻接
            if abs(x_i - x_new) > 1 || abs(y_i - y_new) > 1
                continue;
            end

            % new -> i 是否可行
            if ~edge_valid_rrt_dist(x_new, y_new, x_i, y_i, HeightData)
                continue;
            end

            edge_cost_i = cost_dist(x_new, y_new, x_i, y_i, HeightData);
            if isinf(edge_cost_i)
                continue;
            end

            new_cost_candidate = nodes(node_count).cost + edge_cost_i;
            if new_cost_candidate < nodes(i).cost
                nodes(i).parent = node_count;
                nodes(i).cost   = new_cost_candidate;
            end
        end

        %检查是否可以更新终点
        dist_to_goal = sqrt((x_new - endx)^2 + (y_new - endy)^2);
        if dist_to_goal <= goal_radius
            % 再检查从 x_new 到终点是否可行（仍然只允许邻居）
            if edge_valid_rrt_dist(x_new, y_new, endx, endy, HeightData)
                edge_goal_cost = cost_dist(x_new, y_new, endx, endy, HeightData);
                if ~isinf(edge_goal_cost)
                    total_cost = nodes(node_count).cost + edge_goal_cost;
                    if total_cost < best_length
                        best_length     = total_cost;
                        best_goal_parent= node_count;
                    end
                end
            end
        end

    end % for iter

    % 回溯生成路径  
    if best_goal_parent == -1
        disp('RRT*（只考虑距离）未找到到达终点的路径');
        path = [];
        return;
    end

    % 从 best_goal_parent 回溯到起点
    path = [endx, endy]; % 终点放在最后一个
    idx = best_goal_parent;
    while idx ~= 0
        path = [nodes(idx).x, nodes(idx).y; path];
        idx = nodes(idx).parent;
    end

    path_length = 0;
    for i = 2:size(path,1)
        dx = path(i,1) - path(i-1,1);
        dy = path(i,2) - path(i-1,2);
        dz = HeightData(path(i,2), path(i,1)) - HeightData(path(i-1,2), path(i-1,1));
        path_length = path_length + sqrt(dx^2 + dy^2 + dz^2);
    end

    disp("RRT* (dist) path length (3D): " + path_length);
    disp("RRT* (dist) best_length     : " + best_length);
end


function ok = edge_valid_rrt_dist(x1, y1, x2, y2, HeightData)
    % 只允许 8 邻居边，不考虑坡度，只检查邻接 & 越界
    [LevelGrid, PortGrid] = size(HeightData); %#ok<ASGLU>

    dx = x2 - x1;
    dy = y2 - y1;

    % 只允许 8 邻接，排除自己
    if abs(dx) > 1 || abs(dy) > 1 || (dx == 0 && dy == 0)
        ok = false;
        return;
    end

    % 越界检查
    if x2 < 1 || x2 > PortGrid || y2 < 1 || y2 > LevelGrid
        ok = false;
        return;
    end

    ok = true;
end


function c = cost_dist(x1, y1, x2, y2, HeightData)
    % 3D 欧氏距离（含高度差）
    dx = x2 - x1;
    dy = y2 - y1;
    dz = HeightData(y2, x2) - HeightData(y1, x1);
    c  = sqrt(dx^2 + dy^2 + dz^2);
end
