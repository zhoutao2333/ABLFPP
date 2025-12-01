function [path, total_energy, path_length] = PRM(HeightData, startx, starty, endx, endy, ...
                                                 n_samples, k_neighbors, max_radius)
    % 概率路线图 PRM，用能量代价 cost 作为边权重
    %
    % 输入：
    %   HeightData  - 高度图（matrix，HeightData(y,x)）
    %   startx,y    - 起点
    %   endx,y      - 终点
    %   n_samples   - 随机采样点数量（不包括起终点）
    %   k_neighbors - 每个节点最多连接的近邻数量
    %   max_radius  - 邻居最大连接距离（像素单位）
    %
    % 输出：
    %   path         - 路径上的网格点 [x,y] 列表（节点序列）
    %   total_energy - 路径能量和（沿 path 用 cost 累加）
    %   path_length  - 路径几何长度（3D，含高度变化）

    [LevelGrid, PortGrid] = size(HeightData);

    [m, u, thetaM , thetaB] = parameter(); 

    %采样节点（起点 + 随机点 + 终点）
    nodes = struct('x',{},'y',{});

    % 起点
    nodes(1).x = startx;
    nodes(1).y = starty;
    node_count = 1;

    % 随机采样 n_samples 个中间点
    while node_count < n_samples + 1
        xr = randi(PortGrid);
        yr = randi(LevelGrid);

        % 去重：如果已经有同样的点就跳过
        duplicate = false;
        for i = 1:node_count
            if nodes(i).x == xr && nodes(i).y == yr
                duplicate = true;
                break;
            end
        end
        if duplicate
            continue;
        end

        node_count = node_count + 1;
        nodes(node_count).x = xr;
        nodes(node_count).y = yr;
    end

    % 终点
    node_count = node_count + 1;
    nodes(node_count).x = endx;
    nodes(node_count).y = endy;

    n_nodes = node_count;

    %构建图的邻接矩阵
    W = inf(n_nodes, n_nodes);         % W(i,j) = 边能量，inf 表示不连
    for i = 1:n_nodes
        W(i,i) = 0;
    end

    xs = [nodes.x];
    ys = [nodes.y];

    for i = 1:n_nodes
        xi = xs(i);
        yi = ys(i);

        % 计算到所有其他节点的距离
        dists = sqrt((xs - xi).^2 + (ys - yi).^2);
        [sorted_dists, idxs] = sort(dists);

        % 依次尝试连接最近的 k_neighbors 个（距离也不能超过 max_radius）
        neighbor_num = 0;
        for t = 2:n_nodes   % 从 2 开始是因为 1 是自己（距离为 0）
            j = idxs(t);
            d = sorted_dists(t);

            if d > max_radius
                break;  % 剩下的只会更远
            end

            if W(i,j) < inf
                continue; % 已经连过
            end

            xj = xs(j);
            yj = ys(j);

            % 用 edge_valid_rrt 检查中间是否经过坡度非法区域
            if ~edge_valid_rrt(xi, yi, xj, yj, HeightData, m, u, thetaM, thetaB)
                continue;
            end

            e_cost = cost(xi, yi, xj, yj, HeightData, m, u, thetaM, thetaB);
            if isinf(e_cost)
                continue;
            end

            W(i,j) = e_cost;
            W(j,i) = e_cost;

            neighbor_num = neighbor_num + 1;
            if neighbor_num >= k_neighbors
                break;
            end
        end
    end

    %Dijkstra 搜索起点到终点的最小能量路径
    start_idx = 1;
    goal_idx  = n_nodes;

    [idx_path, dist_arr] = dijkstra_on_graph(W, start_idx, goal_idx);

    if isempty(idx_path)
        disp('PRM: 无法在构建的路线图中找到起点到终点的连通路径');
        path = [];
        total_energy = inf;
        path_length  = inf;
        return;
    end

    % 将节点索引转为 [x,y] 序列
    path = zeros(length(idx_path), 2);
    for k = 1:length(idx_path)
        path(k,1) = nodes(idx_path(k)).x;
        path(k,2) = nodes(idx_path(k)).y;
    end

    %沿 path 重新累加真实能量 & 3D 路径长度
    total_energy = 0;
    path_length  = 0;
    for i = 2:size(path,1)
        x1 = path(i-1,1); y1 = path(i-1,2);
        x2 = path(i,1);   y2 = path(i,2);

        % 3D长度（含高度变化）
        dx = x2 - x1;
        dy = y2 - y1;
        dz = HeightData(y2, x2) - HeightData(y1, x1);  % 注意索引 (y,x)
        path_length = path_length + sqrt(dx^2 + dy^2 + dz^2);

        % 能量
        c_edge = cost(x1, y1, x2, y2, HeightData, m, u, thetaM, thetaB);
        if ~isinf(c_edge)
            total_energy = total_energy + c_edge;
        end
    end

    disp("PRM path length: " + path_length);
    disp("PRM energy     : " + total_energy);
end



function [path_idx, dist] = dijkstra_on_graph(W, start_idx, goal_idx)
    % 简单 Dijkstra，用于 PRM 图上的最短路（能量最小）
    n = size(W,1);
    visited = false(n,1);
    dist    = inf(n,1);
    prev    = zeros(n,1);

    dist(start_idx) = 0;

    for k = 1:n
        % 在未访问节点中选 dist 最小的
        min_val = inf;
        u = -1;
        for i = 1:n
            if ~visited(i) && dist(i) < min_val
                min_val = dist(i);
                u = i;
            end
        end

        if u == -1 || isinf(min_val)
            break;  % 剩余节点不可达
        end

        visited(u) = true;
        if u == goal_idx
            break;
        end

        % 松弛邻居
        neighbors = find(W(u,:) < inf);
        for v = neighbors
            alt = dist(u) + W(u,v);
            if alt < dist(v)
                dist(v) = alt;
                prev(v) = u;
            end
        end
    end

    % 回溯路径
    if isinf(dist(goal_idx))
        path_idx = [];
        return;
    end

    path_idx = goal_idx;
    u = goal_idx;
    while u ~= start_idx
        u = prev(u);
        if u == 0
            path_idx = [];
            return;
        end
        path_idx = [u, path_idx];
    end
end

function ok = edge_valid_rrt(x1, y1, x2, y2, HeightData, m, u, thetaM, thetaB)
    % 沿着 (x1,y1) -> (x2,y2) 线段离散采样，每一小段用 cost() 检查坡度合法性
    [LevelGrid, PortGrid] = size(HeightData);

    N = max(abs(x2 - x1), abs(y2 - y1));
    if N == 0
        ok = true;
        return;
    end

    xs = linspace(x1, x2, N+1);
    ys = linspace(y1, y2, N+1);

    ok = true;
    for k = 1:N
        xa = round(xs(k));
        ya = round(ys(k));
        xb = round(xs(k+1));
        yb = round(ys(k+1));

        % 越界检测（x 是列，y 是行）
        if xa < 1 || xa > PortGrid || xb < 1 || xb > PortGrid || ...
           ya < 1 || ya > LevelGrid || yb < 1 || yb > LevelGrid
            ok = false;
            return;
        end

        c = cost(xa, ya, xb, yb, HeightData, m, u, thetaM, thetaB);
        if isinf(c)
            ok = false;
            return;
        end
    end
end

function c = cost(x1, y1, x2, y2, HeightData, m, u, thetaM , thetaB)
    % 计算移动成本，考虑地形高度与坡度

    dist_xy = sqrt((x2  - x1 )^2 + (y2 - y1)^2);
    if dist_xy == 0
        c = 0;
        return;
    end

    dist_h = HeightData(y2, x2)  - HeightData(y1, x1);
    xxxxx = atan(dist_h / dist_xy);

    if xxxxx > thetaM
        c = Inf; 
    elseif xxxxx > thetaB

        dist = sqrt((x2 - x1)^2 + (y2 - y1)^2 + ...
                      (HeightData(y2, x2) - HeightData(y1, x1))^2);
        c = m * 9.81 * dist * (u * cos(xxxxx) + sin(xxxxx));
    else
        c = 0;
    end
end

