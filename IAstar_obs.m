function path = astar_energypath_final(HeightData,Obstacle_map, startx, starty, endx, endy)
    %% Theta* �����㷨��֧�ַǾ��ȴ��۵�ͼ��
    %  HeightData - ���θ߶Ⱦ���ֵԽ���ʾ����Խ�ߣ�
    %  startx, starty - �������
    %  endx, endy - �յ�����

    % ========== �������� ==========
    % �������������
    m = 22;       % ���� (kg)
    v = 0.35;     % �ٶ� (m/s)
    Pmax = 72;    % ����� (W)
    u = 0.1;      % Ħ��ϵ��
    us = 1.0;     % ��Ħ��ϵ��
    g = 9.81;     % �������ٶ�
    num_cu = 0;

    % �����¶�����
    thetaf = calculateTHm(m, v, Pmax, u);  % ���������¶�
    thetaS = atan(us - u);                 % ��Ħ���¶�
    thetaM = min(thetaf, thetaS);          % �������¶�
    thetaB = -atan(u);                     % ɲ���¶ȣ����£�
    % ========== ��ʼ�� ==========
    % 8�����ƶ�����
    directions = [0 1; 1 0; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1];
    
    % open_list��ʽ: [x, y, f, g, h]
    open_list = [startx, starty, 0, 0, heuristic(startx, starty, endx, endy, HeightData, m, u, thetaM, thetaB),startx, starty];
    closed_list = [];
    parent_map = containers.Map(); % �洢���ڵ��ϵ
    
    % ========== ��ѭ�� ==========
    while ~isempty(open_list)
        % ѡȡfֵ��С�Ľڵ�
        [~, idx] = min(open_list(:, 3));
        current = open_list(idx, :);
        open_list(idx, :) = [];
        closed_list = [closed_list; current(1:2)];
        num_cu = num_cu + 1;
        % ����Ƿ񵽴��յ�
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
 
        % ���������ھ�
        for i = 1:size(directions, 1)
            new_x = current(1) + directions(i, 1);
            new_y = current(2) + directions(i, 2);
            
            % ���Խ��
            if new_x < 1 || new_x > size(HeightData, 2) || new_y < 1 || new_y > size(HeightData, 1)
                continue;
            end

            if Obstacle_map(new_x,new_y) == 1
                continue;
            end            
            
            % ����Ƿ��ѷ���
            if ismember([new_x, new_y], closed_list, 'rows')
                continue;
            end
            
            % ===== Theta* �����Ż� =====
            % �������ֿ��ܵ����ӷ�ʽ
            % ��ʽ1��ͨ����ǰ�ڵ�����
            g1 = current(4) + costx(current(1), current(2), new_x, new_y, HeightData, m, u, thetaM, thetaB);
            % ��ʽ2��ͨ�����ڵ����ӣ�������ڸ��ڵ㣩

            if isKey(parent_map, sprintf('%d_%d', current(1), current(2)))
                parent = parent_map(sprintf('%d_%d', current(1), current(2)));
                [hasLOS, g2] = line_of_sight(Obstacle_map,parent(1), parent(2), new_x, new_y, HeightData, m, u, thetaM, thetaB);
                g2 = g2 + parent(3);
                if hasLOS && g2 <= g1
                    % ѡ�񸸽ڵ�����
                    h_new = heuristic(new_x, new_y, endx, endy, HeightData, m, u, thetaM, thetaB);
                    f_new = g2 + h_new;

                    % ����Ƿ���open_list��
                    existing_idx = find(open_list(:,1) == new_x & open_list(:,2) == new_y);
                    if isempty(existing_idx) || all(g2 < open_list(existing_idx, 4))
                        open_list = [open_list; new_x, new_y, f_new, g2, h_new,parent(1), parent(2)];
                        parent_map(sprintf('%d_%d', new_x, new_y)) = [parent(1), parent(2), parent(3)];
                    end
                    continue; % ������׼����
                end
            end
            % ===== ��׼A*���� =====
            h_new = heuristic(new_x, new_y, endx, endy, HeightData, m, u, thetaM, thetaB);
            f_new = g1 + h_new;
            
            % ����Ƿ���open_list��
            existing_idx = find(open_list(:,1) == new_x & open_list(:,2) == new_y);
            if isempty(existing_idx) || all(g1 < open_list(existing_idx, 4))
                open_list = [open_list; new_x, new_y, f_new, g1, h_new,current(1), current(2)];
                parent_map(sprintf('%d_%d', new_x, new_y)) = [current(1), current(2),current(4)];
            end
        end
    end
    
    % �޿���·��
    path = [];
    disp('δ�ҵ�����·����');
end

function [hasLOS, totalCost] = line_of_sight(Obstacle_map,x1, y1, x2, y2, HeightData, m, u, thetaM, thetaB)
    % ʵ��������ͨ�Ӽ��ʹ��ۼ��㣨����ˮƽ�ʹ�ֱ������
    totalCost = 0;
    % ��ʼ������
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    s_x = sign(x2 - x1);
    s_y = sign(y2 - y1);
    tx = 2 * dx;
    ty = 2 * dy;
    % ��������յ��Ƿ���ͬ
    if dx == 0 && dy == 0
        hasLOS = true;
        return;
    end
    g_x = x2;
    g_y = y2;
    % ѡ��������ˮƽ��ֱ��
    if tx > ty
        % ===== ˮƽ����Ϊ���� =====
        tau = (ty - tx)/2;
        rho = (ty + tx)/2;
        x = x1;
        y = y1;
        e = 0;
        x_temp = x1;
        y_temp = y1;
        % �������Ƿ����
        cell_cost = costx(x, y, x+s_x, y, HeightData, m, u, thetaM, thetaB);
        totalCost = totalCost + 0.5 * cell_cost;
        if isinf(cell_cost)
            hasLOS = false;
            totalCost = Inf;
            return;
        end
        while x ~= x2
            if e > tau
                % x�����ƶ�
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
                % y�����ƶ�
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
                % �Խ����ƶ�
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
        
        % �յ���۵���
        totalCost = totalCost - 0.5 * costx(x2-s_x, y2, x2, y2, HeightData, m, u, thetaM, thetaB);
        if dx ~= 0
            totalCost = totalCost * sqrt(tx^2 + ty^2)/tx;
        end
    elseif tx < ty
        % ===== ��ֱ����Ϊ���� =====
        tau = (tx - ty)/2;
        rho = (tx + ty)/2;
        x = x1;
        y = y1;
        e = 0;
        x_temp = x1;
        y_temp = y1;
        % �������Ƿ����
        cell_cost = costx(x, y, x, y + s_y, HeightData, m, u, thetaM, thetaB);
        if isinf(cell_cost)
            hasLOS = false;
            totalCost = Inf;
            return;
        end
        totalCost = totalCost + 0.5 * cell_cost;
        while y ~= y2
            if e > tau
                % y�����ƶ�
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
                % x�����ƶ�
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
                % �Խ����ƶ�
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
        
        % �յ���۵���
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
    % ���㵥�����ӵĴ���
    if x == x2 && y == y2
        cell_cost = 0;
        return;
    end
    % ��������һ�����ӵ��¶�
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
    % �����ƶ��ɱ������ǵ��θ߶�

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
    % ����ʽ������ʹ��ŷ����þ���
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
    % ��������·��
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
