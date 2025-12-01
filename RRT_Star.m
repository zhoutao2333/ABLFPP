function [path, best_energy] = RRT_Star(HeightData, startx, starty, endx, endy, max_iter, stepsize, goal_radius)
    [LevelGrid, PortGrid] = size(HeightData);

    % 物理与坡度参数
    [m, u, thetaM , thetaB] = parameter(); 

    %RRT* 初始化 --
    % 节点结构：x, y, cost(能量), parent(索引)
    nodes(1).x      = startx;
    nodes(1).y      = starty;
    nodes(1).cost   = 0;
    nodes(1).parent = 0;
    node_count      = 1;

    best_energy      = inf;
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

        %Steer：从 near 向 rand 走一步
        dir = [x_rand - x_near, y_rand - y_near];
        dist = norm(dir);
        if dist == 0
            continue;
        end
        dir = dir / dist;

        step = min(stepsize, dist);
        x_new = x_near + step * dir(1);
        y_new = y_near + step * dir(2);

        % 仍然映射回整数网格
        x_new = round(x_new);
        y_new = round(y_new);

        % 落在同一格子没意义
        if x_new == x_near && y_new == y_near
            continue;
        end

        % 越界检查
        if x_new < 1 || x_new > PortGrid || y_new < 1 || y_new > LevelGrid
            continue;
        end

        %碰撞 / 坡度合法性检查
        if ~edge_valid_rrt(x_near, y_near, x_new, y_new, HeightData, m, u, thetaM, thetaB)
            continue;
        end

        % 边的能量代价
        edge_cost = cost(x_near, y_near, x_new, y_new, HeightData, m, u, thetaM, thetaB);
        if isinf(edge_cost)
            continue;
        end

        %找邻域节点（RRT* 核心）
        % 邻域半径 r(n) ≈ gamma * (log n / n)^(1/2)
        n = node_count;
        r = gamma_rrt * (log(n + 1) / (n + 1))^(1/2);
        dists_new = sqrt((xs - x_new).^2 + (ys - y_new).^2);
        X_near_idx = find(dists_new <= r);

        %选择最优父节点
        best_parent = idx_near;
        best_new_cost = nodes(idx_near).cost + edge_cost;

        for k = 1:length(X_near_idx)
            i = X_near_idx(k);
            x_i = nodes(i).x;
            y_i = nodes(i).y;

            % 检查边 i -> new 是否可行
            if ~edge_valid_rrt(x_i, y_i, x_new, y_new, HeightData, m, u, thetaM, thetaB)
                continue;
            end

            edge_cost_i = cost(x_i, y_i, x_new, y_new, HeightData, m, u, thetaM, thetaB);
            if isinf(edge_cost_i)
                continue;
            end

            new_cost_candidate = nodes(i).cost + edge_cost_i;
            if new_cost_candidate < best_new_cost
                best_new_cost = new_cost_candidate;
                best_parent = i;
            end
        end

        %将 x_new 插入树 --------------------
        node_count = node_count + 1;
        nodes(node_count).x      = x_new;
        nodes(node_count).y      = y_new;
        nodes(node_count).cost   = best_new_cost;
        nodes(node_count).parent = best_parent;

        %-------------------- 8. Rewire 邻居节点 --------------------
        for k = 1:length(X_near_idx)
            i = X_near_idx(k);
            if i == best_parent || i == node_count
                continue;
            end

            x_i = nodes(i).x;
            y_i = nodes(i).y;

            % 检查 new -> i 是否可行
            if ~edge_valid_rrt(x_new, y_new, x_i, y_i, HeightData, m, u, thetaM, thetaB)
                continue;
            end

            edge_cost_i = cost(x_new, y_new, x_i, y_i, HeightData, m, u, thetaM, thetaB);
            if isinf(edge_cost_i)
                continue;
            end

            new_cost_candidate = nodes(node_count).cost + edge_cost_i;
            if new_cost_candidate < nodes(i).cost
                % 简化版：只更新该节点父亲和 cost（不递归更新子树）
                nodes(i).parent = node_count;
                nodes(i).cost   = new_cost_candidate;
            end
        end

        %-------------------- 9. 检查是否可以更新终点 --------------------
        dist_to_goal = sqrt((x_new - endx)^2 + (y_new - endy)^2);
        if dist_to_goal <= goal_radius
            % 再检查从 x_new 到终点是否可行
            if edge_valid_rrt(x_new, y_new, endx, endy, HeightData, m, u, thetaM, thetaB)
                edge_goal_cost = cost(x_new, y_new, endx, endy, HeightData, m, u, thetaM, thetaB);
                if ~isinf(edge_goal_cost)
                    total_cost = nodes(node_count).cost + edge_goal_cost;
                    if total_cost < best_energy
                        best_energy = total_cost;
                        best_goal_parent = node_count;
                    end
                end
            end
        end

    end % for iter

    %-------------------- 10. 回溯生成路径 --------------------
    if best_goal_parent == -1
        disp('RRT* 未找到到达终点的路径');
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

    % 计算路径真实空间长度（含高度）
    path_length = 0;
    for i = 2:size(path,1)
        dx = path(i,1) - path(i-1,1);
        dy = path(i,2) - path(i-1,2);
        dz = HeightData(path(i,2), path(i,1)) - HeightData(path(i-1,2), path(i-1,1));
        path_length = path_length + sqrt(dx^2 + dy^2 + dz^2);
    end

    disp("RRT* path length: " + path_length);
    disp("RRT* energy     : " + best_energy);

end


function ok = edge_valid_rrt(x1, y1, x2, y2, HeightData, m, u, thetaM, thetaB)
    % 沿着 (x1,y1) -> (x2,y2) 线段离散采样，每一个小段都用你的 cost() 检查坡度
    [LevelGrid, PortGrid] = size(HeightData);

    % 按最大坐标差来确定离散步数
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

        % 越界
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


function c = cost(x1, y1, x2, y2, HeightData, m,u, thetaM , thetaB)
    % 计算移动成本，考虑地形高

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

function thetam = calculateTHm(m,v,Pmax,u)
    Fmax = Pmax / v;
    temp = asin(Fmax / (m * 9.81 * sqrt(u*u + 1)));
    thetam = temp - atan(u);
end