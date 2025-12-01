function [paths, costs,len] = TSP_MultiAstar_FRA_AH_no_history(HeightData, waypoints)
    % 使用 FRA* 计算所有点对路径
    num_points = size(waypoints, 1);
    paths = cell(num_points, num_points);
    costs = zeros(num_points, num_points);
    len = zeros(num_points, num_points);
    N = size(HeightData,1);
    % 初始化 FRA* 状态
    g = inf(N); 
    parent = zeros(N,N,2);
    Closed = false(N); 
    Open = [];

    [m, u, thetaM , thetaB] = parameter(); 
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
                % 第一次：运行完整 A*
            [path, Open, Closed, g, parent,costA,path_length] = Astar(HeightData, start, goal, g, parent, m, u, thetaM, thetaB);


            % 保存结果
            paths{i,j} = path;
            %paths{j,i} = flipud(path); % 反向路径
            if isempty(path)
                costs(i,j) = inf;
            else
                costs(i,j) = costA;
                len(i,j) = path_length;
            end
        end
    end
end



function [path, Open, Closed, g, parent,costA,path_length] = Astar(HeightData, start, goal, g, parent, m, u, thetaM, thetaB)
    num_cu = 0;
    Closed = false(size(HeightData));
    Open = [start heuristic(start(1),start(2),goal(1),goal(2),HeightData, m, u, thetaM, thetaB)];
    g(:) = inf;
    g(start(1),start(2)) = 0;
    cost_map = containers.Map;
    while ~isempty(Open)
        [~,idx] = min(Open(:,3));
        cur = Open(idx,1:2);
        Open(idx,:) = [];
        num_cu = num_cu + 1;       
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
            if Closed(nb(1),nb(2)), continue; end
            ng = g(cur(1),cur(2))+cost(cur(1),cur(2),nb(1),nb(2),HeightData, m, u, thetaM, thetaB);
            if ng < g(nb(1),nb(2))
                g(nb(1),nb(2)) = ng;
                parent(nb(1),nb(2),:) = cur;
                f = ng+heuristic(nb(1),nb(2),goal(1),goal(2),HeightData, m, u, thetaM, thetaB);
                Open = [Open; nb f];
            end
        end
    end

    path = [];
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


function path = ReconstructPath(parent,start,goal)
    path = goal;
    cur = goal;
    while ~isequal(cur,start)
        p = squeeze(parent(cur(1),cur(2),:))';
        if all(p==0), break; end
        path = [p; path];
        cur = p;
    end
end
