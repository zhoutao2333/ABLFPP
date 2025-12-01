%====MSD-GWO TEST for TSPLIB, which in TSP_DATA====%

N=100;    % 种群数       
Max_iter = 2000;    % 迭代次数
perturbation = 40;  %扰动值

city = read_tsp_coords('data/att48.tsp');


City = city;
M = size(City, 1);
dim = M;
disp("it's dim")
disp(dim)

dist_matrix = zeros(M, M);  % 初始化距离矩阵
for i = 1:M
    for j = i+1:M
        dist = round(sqrt(sum((City(i, :) - City(j, :)).^2)));  % 计算两城市间的距离
        dist_matrix(i, j) = dist;
        dist_matrix(j, i) = dist;  
    end
end

his_best = [0,0,0,0,0,0,0,0,0,0];
for tbs = 1:10

Alpha_pos = [];
Alpha_score = inf;
Beta_pos = [];
Beta_score = inf;
Delta_pos = [];
Delta_score = inf;
Best_pos = [];
Best_score = inf;

%118777.3007
Positions = zeros(N, dim);
bandw = ceil(min(M,N) * 0.24);
bandn = ceil(min(M,N) * 0.24);
disp(bandw)

Positions(1:bandn,:) = greedy_beam_search_multi(city,bandw,bandn);

for i = bandn + 1 : N
    Positions(i,:) = randperm(M);
end

count = 0;

Length_best = zeros(1, Max_iter);
l = 1;
tic
while l <= Max_iter
    for i = 1:N
        fitness = Fun(Positions(i,:), dist_matrix, M);
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
    a = 2-l*((2)/Max_iter);
    % 更新个体
    for i = 1:N
        r1 = rand(); 
        A1 = 2*a*r1-a;                   
                   
        r1 = rand();    
        A2 = 2*a*r1-a;                                 
        
        r1 = rand();    
        A3 = 2*a*r1-a;               
        Xi = Positions(i,:);
        
        hd_a = hamming_dist(Xi, Alpha_pos);
        hd_b = hamming_dist(Xi, Beta_pos);
        hd_d = hamming_dist(Xi, Delta_pos);
        hisfit = Fun(Xi, dist_matrix, M);
        if abs(A1) > 1
            X1 = shift(Xi, dist_matrix, M, hd_a, hisfit);
        else
            X1 = swp(Xi, dist_matrix, M, hd_a, hisfit);
        end
        if abs(A2) > 1
            X2 = shift(Xi, dist_matrix, M, hd_b, hisfit);
        else
            X2 = swp(Xi, dist_matrix, M, hd_b, hisfit);
        end
        if abs(A3) > 1
            X3 = shift(Xi, dist_matrix, M, hd_d, hisfit);
        else
            X3 = swp(Xi, dist_matrix, M, hd_d, hisfit);
        end
        Xa = two_opt(X1, dist_matrix, M, hd_a, hisfit);
        Xb = two_opt(X2, dist_matrix, M, hd_b, hisfit);
        Xd = two_opt(X3, dist_matrix, M, hd_d, hisfit);
        f1 = Fun(Xa,dist_matrix,M);
        f2 = Fun(Xb,dist_matrix,M);
        f3 = Fun(Xd,dist_matrix,M);
        [fffff, best_idx] = min([f1, f2, f3]);
        r3 = rand();   
 

        switch best_idx
            case 1
                    Positions(i,:) = Xa;
            case 2
                    Positions(i,:) = Xb;
            case 3
                    Positions(i,:) = Xd;
        end

        if count == perturbation
            disp("distrub")
            if r3 < 0.5
               Positions(i,:) = gachange(Alpha_pos, Beta_pos);
               Positions(i,:) = swp2(Positions(i,:),1);
            end
        end
        if count == perturbation + 1
            count = 0;
        end

    end
    Length_best(l) = Alpha_score;
    disp(['times   ' num2str(tbs) '   Iteration ' num2str(l) ': Best Fitness = ' num2str(Alpha_score)]);
    his_best(tbs) = Alpha_score;
    l = l + 1;
end
end
disp("it is 10")
disp(his_best)
BestSol = Alpha_pos;
disp(Alpha_pos)
figure;
for i=1:M-1
    plot([City(BestSol(i),1),City(BestSol(i+1),1)], [City(BestSol(i),2),City(BestSol(i+1),2)], 'ro-');
    hold on;
end
plot([City(BestSol(end),1),City(BestSol(1),1)], [City(BestSol(end),2),City(BestSol(1),2)], 'ro-');
title('最优路线');
xlabel('X'); ylabel('Y');

figure;
disp("均值：" + mean(his_best))
plot(1:Max_iter, Length_best, 'b-', 'LineWidth', 2);
xlabel('迭代次数');
ylabel('最短路径');
title('收敛过程');
%109389.1383


function d = hamming_dist(a, b)
    d = sum(a ~= b);
end

%% 2-opt
function new_sol = two_opt(sol, dist_matrix, M,k, hisfit)
    new_sol = sol;
    best_sol = sol;
    for t = 1:k
        tem_sol = new_sol;
        i = randi(length(sol)-1);
        j = randi([i+1, length(sol)]);
        tem_sol(i:j) = tem_sol(j:-1:i); 
        tem_fit = Fun(tem_sol, dist_matrix,M);
        if tem_fit < hisfit
            new_sol = tem_sol;
            hisfit = tem_fit;
        end
    end
end


function new_sol = shift(sol, City, M,k, hisfit)
    new_sol = sol;
    for i = 1:k
        a1 = randi([2,length(sol)-1]);
        a2 = randi([a1+1,M]);
        temp = new_sol(a1:a2);
        if a1 < 3
            temp2 = [];
        else
            temp2 = new_sol(1:a1-2);
        end
        newtemp = [temp2,temp];
        newtemp2 = [new_sol(a1-1),new_sol(a2+1:M)];
        temp_sol = [newtemp,newtemp2];
        tem_fit = Fun(temp_sol, City,M);
        if tem_fit < hisfit
            new_sol = temp_sol;
            hisfit = tem_fit;
        end
    end
end

function new_sol = swp(sol, City, M,k, hisfit)
    new_sol = sol;
    for i = 1:k
        a1 = randi(length(sol));
        a2 = randi(length(sol));
        temp_sol = new_sol;
        while a1 == a2
            a1 = randi(length(sol));
            a2 = randi(length(sol));   
        end
        temp = temp_sol(a1);
        temp_sol(a1) = temp_sol(a2);
        temp_sol(a2) = temp;
        tem_fit = Fun(temp_sol, City,M);
        if tem_fit < hisfit
            new_sol = temp_sol;
            hisfit = tem_fit;
        end
    end
end


function new_sol = swp2(sol,k)
    new_sol = sol;
    for i = 1:k
        a1 = randi(length(sol));
        a2 = randi(length(sol));
        temp_sol = new_sol;
        while a1 == a2
            a1 = randi(length(sol));
            a2 = randi(length(sol));   
        end
        temp = temp_sol(a1);
        temp_sol(a1) = temp_sol(a2);
        temp_sol(a2) = temp;
        new_sol = temp_sol;
    end
end

function new_sol = shift2(sol,k)
    new_sol = sol;
    M = size(sol,2);
    for i = 1:k
        a1 = randi([2,length(sol)-1]);
        a2 = randi([a1+1,M]);
        temp = new_sol(a1:a2);
        if a1 < 3
            temp2 = [];
        else
            temp2 = new_sol(1:a1-2);
        end
        newtemp = [temp2,temp];
        newtemp2 = [new_sol(a1-1),new_sol(a2+1:M)];
        temp_sol = [newtemp,newtemp2];
        new_sol = temp_sol;
    end                                                                                                                                                                                                               
end


% 更新代价函数 Fun，使用预计算的距离矩阵
function len = Fun(sol, dist_matrix, M)
    len = 0;
    for k = 1:M-1
        len = len + dist_matrix(sol(k), sol(k+1));  % 直接从距离矩阵查找距离
    end
    len = len + dist_matrix(sol(M), sol(1));  % 闭环，最后一个城市到第一个城市的距离
end


function paths = greedy_beam_search_multi(city, beam_width, branch_factor)
    M = size(city, 1);  
    start_city = randi(M);  
    PathSet = {start_city}; 

    while true
        NewPaths = {};
        for i = 1:length(PathSet)
            path = PathSet{i};
            last_city = path(end);

            remaining = setdiff(1:M, path);

            dists = vecnorm(city(remaining, :) - city(last_city, :), 2, 2);

            [~, idx] = sort(dists);
            next_cities = remaining(idx(1:min(branch_factor, length(idx))));
            for c = next_cities
                NewPaths{end+1} = [path, c];
            end
        end
        scores = cellfun(@(p) path_cost(p, city), NewPaths);
        [~, idx] = sort(scores);
        PathSet = NewPaths(idx(1:min(beam_width, length(idx))));
        finished = all(cellfun(@(x) length(x) == M, NewPaths));
        if finished
            break;
        end

        % 保留 beam_width 条路径

    end

    paths = zeros(beam_width, M);

    for i = 1:beam_width
        paths(i, :) = PathSet{i};
    end
end

function cost = path_cost(path, city)                                           
    cost = 0;
    for i = 1:length(path)-1
        cost = cost + norm(city(path(i), :) - city(path(i+1), :));
    end
    cost = cost + norm(city(path(end), :) - city(path(1), :)); 
end

function child = gachange(p1, p2)
r1 = randi([1,length(p1)-1]);
r2 = randi([r1+1,length(p1)]);
child = p1;
child(r1:r2) = p2(r1:r2);
original_segment = p1(r1:r2);
new_segment = p2(r1:r2);

for i = [1:r1-1, r2+1:length(p1)]
    if any(child(i) == new_segment)
        child(i) = '_'; 
    end
end

fill_idx = 1;  
for i = 1:length(child)
    if child(i) == '_'
        while fill_idx <= length(original_segment)
            candidate = original_segment(fill_idx);
            if ~any(child == candidate)
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

