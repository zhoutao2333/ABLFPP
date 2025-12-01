function [route, total_cost] = TSP_Greedy(cost_matrix, num_points)
% TSP_Greedy - 最近邻贪心，固定起点=1
% 输入:
%   cost_matrix - 点对代价矩阵（方阵，cost_matrix(i,j) 是 i->j 代价）
%   num_points  - 城市数量
% 输出:
%   route       - 访问顺序（以 1 开头，包含所有城市）
%   total_cost  - 总路程（含回到起点）

    start_city = 1;
    route = zeros(1, num_points);
    route(1) = start_city;

    unvisited = true(1, num_points);
    unvisited(start_city) = false;

    cur = start_city;
    for k = 2:num_points
        % 在未访问集合中，选择代价最小的下一城
        costs = cost_matrix(cur, :);
        costs(~unvisited) = inf;      % 只在未访问中找最小
        [~, nxt] = min(costs);
        route(k) = nxt;
        unvisited(nxt) = false;
        cur = nxt;
    end

    % 计算总代价（含回到起点）
    total_cost = 0;
    for k = 1:num_points-1
        total_cost = total_cost + cost_matrix(route(k), route(k+1));
    end
    total_cost = total_cost + cost_matrix(route(end), route(1));
    disp(num2str(total_cost));
end
