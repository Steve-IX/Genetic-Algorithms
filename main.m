% Optimized Genetic Algorithm for Path Planning 
clc;
clear;
close all;

% Load map
map = im2bw(imread('random_map.bmp')); % Binary map: 1 (free space), 0 (obstacles)
[start_x, start_y] = deal(1, 1); % Starting point
[goal_x, goal_y] = deal(size(map, 1), size(map, 2)); % Goal point
noOfPointsInSolution = 8; % Maximum number of turns (path points)

% Parameters
populationSize = 150; % population size
numGenerations = 150; % number of generations
mutationRate = 0.05; % Reduced mutation rate

% Prompt user for methods
selectionMethod = input('Enter selection method (0: RWS, 1: Tournament, 2: Rank-based): ');
crossoverMethod = input('Enter crossover method (0: Arithmetic Crossover, 1: Uniform Crossover): ');
mutationMethod = input('Enter mutation method (0: Random Mutation, 1: Swap Mutation): ');

% Initialize population
population = rand(populationSize, noOfPointsInSolution * 2);

tic; % Start timing
for gen = 1:numGenerations
    % Evaluate fitness
    fitness = arrayfun(@(idx) evaluate_fitness(population(idx,:), map, [start_x, start_y], [goal_x, goal_y]), 1:populationSize)';
    
    % Selection
    switch selectionMethod
        case 0 % Roulette-wheel
            selectedPopulation = roulette_wheel_selection(population, fitness);
        case 1 % Tournament
            selectedPopulation = tournament_selection(population, fitness, 3); 
        case 2 % Rank-based
            selectedPopulation = rank_based_selection(population, fitness);
    end
    
    % Crossover
    parents1 = selectedPopulation(1:2:end, :);
    parents2 = selectedPopulation(2:2:end, :);
    
    if crossoverMethod == 0
        [child1, child2] = arrayfun(@(i) arithmetic_crossover(parents1(i,:), parents2(i,:)), ...
                                    (1:size(parents1,1))', 'UniformOutput', false);
    else
        [child1, child2] = arrayfun(@(i) uniform_crossover(parents1(i,:), parents2(i,:)), ...
                                    (1:size(parents1,1))', 'UniformOutput', false);
    end
    
    child1Mat = vertcat(child1{:});
    child2Mat = vertcat(child2{:});
    offspring = [child1Mat; child2Mat];  % populationSize x (noOfPointsInSolution*2)
    
    % Mutation
    if mutationMethod == 0
        mutatedPop = arrayfun(@(idx) random_mutation(offspring(idx, :), mutationRate), 1:populationSize, 'UniformOutput', false);
    else
        mutatedPop = arrayfun(@(idx) swap_mutation(offspring(idx, :), mutationRate), 1:populationSize, 'UniformOutput', false);
    end
    offspring = vertcat(mutatedPop{:});
    
    % Update population
    population = offspring;
end

% Final evaluation
fitness = arrayfun(@(idx) evaluate_fitness(population(idx,:), map, [start_x, start_y], [goal_x, goal_y]), 1:populationSize)';
[~, bestIdx] = max(fitness);
bestSolution = population(bestIdx, :);

% Denormalize solution to map size
finalPath = [start_x, start_y; ...
    reshape(bestSolution, 2, [])' .* [size(map, 1), size(map, 2)]; ...
    goal_x, goal_y];

% Path smoothing
finalPath = smooth_path(finalPath, map);

% Display final results
executionTime = toc; % End timing
totalDistance = compute_path_length(finalPath);
disp(['Execution Time: ', num2str(executionTime), ' seconds']);
disp(['Total Distance: ', num2str(totalDistance)]);

% Plot final path
imshow(map);
rectangle('position', [1 1 size(map)-1], 'edgecolor', 'k');
line(finalPath(:, 2), finalPath(:, 1), 'Color', 'b', 'LineWidth', 2);
title('Optimized Final Path');

%% Function Definitions

% Fitness evaluation
function fitness = evaluate_fitness(solution, map, start, goal)
    path = [start; ...
        reshape(solution, 2, [])' .* [size(map, 1), size(map, 2)]; ...
        goal];
    pathLength = compute_path_length(path);
    collisionPenalty = check_collisions(path, map);
    fitness = 1 / (pathLength + collisionPenalty * 1e3); % Heavy penalty for collisions
end

% Path length computation
function pathLength = compute_path_length(path)
    diffs = diff(path);
    pathLength = sum(sqrt(sum(diffs.^2, 2)));
end

% Collision checking
function collisionPenalty = check_collisions(path, map)
    nSegments = size(path,1)-1;
    allPoints = arrayfun(@(seg) interpolate_line(path(seg,:), path(seg+1,:)), ...
        (1:nSegments)', 'UniformOutput', false);
    allPoints = vertcat(allPoints{:});
    x = round(allPoints(:,1));
    y = round(allPoints(:,2));

    inside = (x >= 1 & y >= 1 & x <= size(map,1) & y <= size(map,2));
    collisionPenalty = sum(~inside); % Points outside map
    
    insideIndices = find(inside);
    if ~isempty(insideIndices)
        linearIdx = sub2ind(size(map), x(insideIndices), y(insideIndices));
        collisionPenalty = collisionPenalty + sum(map(linearIdx) == 0);
    end
end

% Line interpolation
function points = interpolate_line(p1, p2)
    x = linspace(p1(1), p2(1), 100);
    y = linspace(p1(2), p2(2), 100);
    points = [x', y'];
end

% Path smoothing
function smoothedPath = smooth_path(path, map)
    smoothedPath = path(1, :);
    for i = 2:size(path, 1)-1
        if ~check_collisions([smoothedPath(end, :); path(i+1, :)], map)
            continue;
        else
            smoothedPath = [smoothedPath; path(i, :)];
        end
    end
    smoothedPath = [smoothedPath; path(end, :)];
end

% Roulette-wheel selection
function selected = roulette_wheel_selection(population, fitness)
    probabilities = fitness(:) / sum(fitness);
    cumulativeProb = cumsum(probabilities);
    r = rand(size(population,1),1);
    [~,idx] = histc(r,[0; cumulativeProb]);
    idx(idx==0) = 1; 
    selected = population(idx, :);
end

% Tournament selection
function selected = tournament_selection(population, fitness, k)
    popSize = size(population,1);
    competitors = randi(popSize, [popSize, k]);
    [~, bestIdx] = max(fitness(competitors), [], 2);
    linearIdx = sub2ind([popSize, k], (1:popSize)', bestIdx);
    selectedIndices = competitors(linearIdx);
    selected = population(selectedIndices, :);
end

% Rank-based selection
function selected = rank_based_selection(population, fitness)
    [~, sortedIdx] = sort(fitness, 'descend');
    ranks = (size(population, 1):-1:1)'; % Make sure ranks is a column vector
    probabilities = ranks / sum(ranks);
    cumulativeProb = cumsum(probabilities);
    r = rand(size(population,1), 1);
    [~, idx] = histc(r,[0; cumulativeProb]);
    idx(idx==0) = 1;
    selected = population(sortedIdx(idx),:);
end

% Arithmetic crossover
function [child1, child2] = arithmetic_crossover(parent1, parent2)
    alpha = rand;
    child1 = alpha * parent1 + (1 - alpha) * parent2;
    child2 = alpha * parent2 + (1 - alpha) * parent1;
end

% Uniform crossover
function [child1, child2] = uniform_crossover(parent1, parent2)
    mask = rand(size(parent1)) > 0.5;
    child1 = parent1;
    child1(mask) = parent2(mask);
    child2 = parent2;
    child2(mask) = parent1(mask);
end

% Random mutation
function mutated = random_mutation(individual, mutationRate)
    mutationMask = rand(size(individual)) < mutationRate;
    individual(mutationMask) = rand(sum(mutationMask),1);
    mutated = individual;
end

% Swap mutation
function mutated = swap_mutation(individual, mutationRate)
    mutated = individual;
    if rand < mutationRate && length(individual) > 1
        idx = randperm(length(individual), 2);
        mutated(idx) = mutated(fliplr(idx));
    end
end
