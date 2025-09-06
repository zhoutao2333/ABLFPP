function [Alpha_pos, Alpha_score] = TSP_GWO(cost_matrix, num_points, N, Max_iter)
% GWO_TSP - ʹ�û����Ż��㷨���TSP����˳���Ż����⣬�̶����Ϊ����1
% ����:
%   cost_matrix - ���е��֮���·�����۾���
%   num_points - ��ĸ���
%   N - ���Ǹ�������
%   Max_iter - ����������
% ���:
%   Alpha_pos - ���Ž�λ�ã�����˳�������λ����������1��ͷ��
%   Alpha_score - ����·���ܴ���

    % ��ʼ����Ⱥ��ȷ��ÿ��·���Գ���1��ͷ
    Positions = zeros(N, num_points);
    for i = 1:N
        % ����1�̶�Ϊ��㣬ʣ������������
        Positions(i, :) = [1, randperm(num_points-1)+1];
    end
    disp("i am gwo")

    Alpha_pos = [];
    Alpha_score = inf;
    Beta_pos = [];
    Beta_score = inf;
    Delta_pos = [];
    Delta_score = inf;
    
    bandw = ceil(min(N, num_points) * 0.1);
    bandn = ceil(min(N, num_points) * 0.1);
    disp(bandw)
    
    % ����������
    count = 0;
    Length_best = zeros(1, Max_iter);
    l = 1;

    while l <= Max_iter
        for i = 1:N
            fitness = Fun(Positions(i,:), cost_matrix, num_points);
            if fitness < Alpha_score
                count = 0;
                Alpha_score = fitness;
                Alpha_pos = Positions(i,:);
            elseif fitness < Beta_score
                Beta_score = fitness;
                Beta_pos = Positions(i,:);
            elseif fitness < Delta_score
                Delta_score = fitness;
                Delta_pos = Positions(i,:);
            end
        end
        count = count + 1;
        a = 2 - l * ((2) / Max_iter);
        
        % ���¸���
        for i = 1:N
            r1 = rand(); 
            A1 = 2 * a * r1 - a;                   
            r1 = rand();    
            A2 = 2 * a * r1 - a;                                 
            r1 = rand();    
            A3 = 2 * a * r1 - a;               
            Xi = Positions(i,:);
            
            hd_a = hamming_dist(Xi, Alpha_pos);
            hd_b = hamming_dist(Xi, Beta_pos);
            hd_d = hamming_dist(Xi, Delta_pos);
          
            if abs(A1) > 1
                X1 = shift(Xi, cost_matrix, num_points, hd_a);
            else
                X1 = swp(Xi, cost_matrix, num_points, hd_a);
            end
            if abs(A2) > 1
                X2 = shift(Xi, cost_matrix, num_points, hd_b);
            else
                X2 = swp(Xi, cost_matrix, num_points, hd_b);
            end
            if abs(A3) > 1
                X3 = shift(Xi, cost_matrix, num_points, hd_d);
            else
                X3 = swp(Xi, cost_matrix, num_points, hd_d);
            end
            Xa = two_opt(X1, cost_matrix, num_points, hd_a);
            Xb = two_opt(X2, cost_matrix, num_points, hd_b);
            Xd = two_opt(X3, cost_matrix, num_points, hd_d);
            f1 = Fun(Xa, cost_matrix, num_points);
            f2 = Fun(Xb, cost_matrix, num_points);
            f3 = Fun(Xd, cost_matrix, num_points);
            [~, best_idx] = min([f1, f2, f3]);
            r3 = rand();   
            xa = gachange(Alpha_pos, Beta_pos, num_points);
            xb = gachange(Beta_pos, Delta_pos, num_points);
            xd = gachange(Alpha_pos, Delta_pos, num_points);
            fa = Fun(xa, cost_matrix, num_points);
            fb = Fun(xb, cost_matrix, num_points);
            fc = Fun(xd, cost_matrix, num_points);
            [~, best_idx2] = min([fa, fb, fc]);
            
            switch best_idx
                case 1
                    Positions(i,:) = Xa;
                case 2
                    Positions(i,:) = Xb;
                case 3
                    Positions(i,:) = Xd;
            end
    
            if count == 20
                if r3 < 0.5
                    Positions(i,:) = gachange(Alpha_pos, Beta_pos, num_points);
                    Positions(i,:) = swp2(Positions(i,:), 1, num_points);
                end
            end
            if count == 60
                count = 0;
            end
            if Alpha_score < 8.7760
                break;
            end
        end
        Length_best(l) = Alpha_score;
        disp(['Iteration ' num2str(l) ': Best Fitness = ' num2str(Alpha_score)]);
        %disp(Alpha_pos)
        l = l + 1;
    end
end

%% ����·�����۵ĺ���
function len = Fun(sol, cost_matrix, num_points)
    len = 0;
    for k = 1:num_points-1
        len = len + cost_matrix(sol(k), sol(k+1));
    end
    % ��ӻص����Ĵ���
    len = len + cost_matrix(sol(num_points), sol(1));
end

%% ��������
function d = hamming_dist(a, b)
    d = sum(a ~= b);
end

%% 2-opt�Ż����������Ϊ1
function new_sol = two_opt(sol, cost_matrix, num_points, k)
    new_sol = sol;
    for t = 1:k
        tem_sol = new_sol;
        % ���������㣨����1����������2��ʼ
        i = randi([2, num_points-1]);
        j = randi([i+1, num_points]);
        tem_sol(i:j) = tem_sol(j:-1:i);
        if Fun(tem_sol, cost_matrix, num_points) < Fun(new_sol, cost_matrix, num_points)
            new_sol = tem_sol;
        end
    end
end

%% Shift�������������Ϊ1
function new_sol = shift(sol, cost_matrix, num_points, k)
    new_sol = sol;
    for i = 1:k
        % ������2��ʼ�������ƶ����
        a1 = randi([2, num_points-1]);
        a2 = randi([a1+1, num_points]);
        temp = new_sol(a1:a2);
        if a1 < 3
            temp2 = [];
        else
            temp2 = new_sol(2:a1-1);
        end
        newtemp = [new_sol(1), temp2, temp];
        newtemp2 = new_sol(a2+1:num_points);
        temp_sol = [newtemp, newtemp2];
        if Fun(temp_sol, cost_matrix, num_points) < Fun(new_sol, cost_matrix, num_points)
            new_sol = temp_sol;
        end
    end
end

%% Swap�������������Ϊ1
function new_sol = swp(sol, cost_matrix, num_points, k)
    new_sol = sol;
    for i = 1:k
        % ���⽻����㣨����1��
        a1 = randi([2, num_points]);
        a2 = randi([2, num_points]);
        while a1 == a2
            a1 = randi([2, num_points]);
            a2 = randi([2, num_points]);   
        end
        temp_sol = new_sol;
        temp = temp_sol(a1);
        temp_sol(a1) = temp_sol(a2);
        temp_sol(a2) = temp;
        if Fun(temp_sol, cost_matrix, num_points) < Fun(new_sol, cost_matrix, num_points)
            new_sol = temp_sol;
        end
    end
end

%% Swap2�������������Ϊ1
function new_sol = swp2(sol, k, num_points)
    new_sol = sol;
    for i = 1:k
        % ���⽻����㣨����1��
        a1 = randi([2, num_points]);
        a2 = randi([2, num_points]);
        while a1 == a2
            a1 = randi([2, num_points]);
            a2 = randi([2, num_points]);   
        end
        temp_sol = new_sol;
        temp = temp_sol(a1);
        temp_sol(a1) = temp_sol(a2);
        temp_sol(a2) = temp;
        new_sol = temp_sol;
    end
end

%% Shift2�������������Ϊ1
function new_sol = shift2(sol, k, num_points)
    new_sol = sol;
    for i = 1:k
        % ������2��ʼ�������ƶ����
        a1 = randi([2, num_points-1]);
        a2 = randi([a1+1, num_points]);
        temp = new_sol(a1:a2);
        if a1 < 3
            temp2 = [];
        else
            temp2 = new_sol(2:a1-1);
        end
        newtemp = [new_sol(1), temp2, temp];
        newtemp2 = new_sol(a2+1:num_points);
        temp_sol = [newtemp, newtemp2];
        new_sol = temp_sol;
    end
end

%% ����������������Ϊ1
function child = gachange(p1, p2, num_points)
    r1 = randi([2, num_points-1]); % ������2��ʼ�������޸����
    r2 = randi([r1+1, num_points]);
    child = p1;
    child(r1:r2) = p2(r1:r2);
    original_segment = p1(r1:r2);
    new_segment = p2(r1:r2);

    % ����ظ�����
    for i = [2:r1-1, r2+1:num_points] % �������
        if any(child(i) == new_segment)
            child(i) = '_';
        end
    end

    % ��ԭʼƬ�����ȱʧ�ĳ���
    fill_idx = 1;
    for i = 2:num_points % �������
        if child(i) == '_'
            while fill_idx <= length(original_segment)
                candidate = original_segment(fill_idx);
                if ~any(child(2:end) == candidate) % �������λ��
                    child(i) = candidate;
                    fill_idx = fill_idx + 1;
                    break;
                else
                    fill_idx = fill_idx + 1;
                end
            end
        end
    end
end

%% Greedy Beam Search���̶����Ϊ1
function paths = greedy_beam_search_multi(num_points, cost_matrix, beam_width, branch_factor)
    M = num_points;
    start_city = 1; % �̶����Ϊ����1
    PathSet = {start_city};

    while true
        NewPaths = {};
        for i = 1:length(PathSet)
            path = PathSet{i};
            last_city = path(end);
            remaining = setdiff(1:M, path);
            dists = cost_matrix(last_city, remaining);
            [~, idx] = sort(dists);
            next_cities = remaining(idx(1:min(branch_factor, length(idx))));
            for c = next_cities
                NewPaths{end+1} = [path, c];
            end
        end
        scores = cellfun(@(p) Fun(p, cost_matrix, num_points), NewPaths);
        [~, idx] = sort(scores);
        PathSet = NewPaths(idx(1:min(beam_width, length(idx))));
        finished = all(cellfun(@(x) length(x) == M, NewPaths));
        if finished
            break;
        end
    end

    paths = zeros(beam_width, M);
    for i = 1:min(beam_width, length(PathSet))
        paths(i, :) = PathSet{i};
    end
end

%% ����·�����ۣ�����greedy_beam_search_multi��
function cost = path_cost(path, city)
    cost = 0;
    for i = 1:length(path)-1
        cost = cost + norm(city(path(i), :) - city(path(i+1), :));
    end
    cost = cost + norm(city(path(end), :) - city(path(1), :)); % �����
end