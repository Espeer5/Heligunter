classdef PRM
    %  PRM - A simple naive PRM planner for 3D course planning
    
    properties
        sample_n double
        sample_points double
        K double
        space world
        adjmat uint8
        path double
    end

    methods

        function PRM = PRM(sample_n, K, space)
           % Construct a PRM planner
           PRM.sample_n = sample_n;
           PRM.K = K;
           PRM.space = space;
           PRM.adjmat = zeros(PRM.sample_n);
        end

        function PRM = sampler(PRM)
            % Sample the planner's space uniformly and store the result
            new_sample_points = zeros(PRM.sample_n, 3);
            index = 1;
            added = 0;
            while added < PRM.sample_n
                x = randi(PRM.space.xmax);
                y = randi(PRM.space.ymax);
                z = randi(PRM.space.zmax);
                sample = [x, y, z];
                if in_freespace(PRM.space, sample)
                    new_sample_points(index, 1:3) = sample;
                    added = added + 1;
                    index = index + 1;
                end
            end
            PRM.sample_points = new_sample_points;
        end

        function show_sample(PRM)
            plot_world(PRM.space)
            scatter3(PRM.sample_points(:, 1), PRM.sample_points(:, 2), ...
                PRM.sample_points(:, 3), "filled", "blue")
        end

        function PRM = create_edges(PRM)
            % Use a KDtree to generate edges between K nearest neighbors
            kdt = KDTreeSearcher(PRM.sample_points, 'BucketSize', PRM.K);
            extra = 3;
            for i = 1:PRM.sample_n
                neighbors = knnsearch(kdt, PRM.sample_points(i, :), 'K', ...
                    PRM.K + extra + 1);
                found = 0;
                j = 1;
                while found < PRM.K
                    j = j + 1;
                    if j > PRM.K + extra
                        neighbors = knnsearch(kdt, PRM.sample_points(i, :), ...
                            'K', PRM.K + j + 1);
                    end
                    point1 = PRM.sample_points(neighbors(j), :);
                    point2 = PRM.sample_points(i, :);
                    if connects(PRM.space, point1, point2)
                        dist = norm(point2 - point1);
                        PRM.adjmat(i, neighbors(j)) = dist;
                        PRM.adjmat(neighbors(j), i) = dist;
                        found = found + 1;
                    end
                end
            end
        end

        function show_graph(PRM)
            % Plot the graph of the PRM with connected edges minus the path
            show_sample(PRM)
            for i=1:PRM.sample_n
                edges = 0;
                j = 0;
                while edges < PRM.K
                    j = j + 1;
                    if PRM.adjmat(i, j) == 1
                        point1 = PRM.sample_points(i, :);
                        point2 = PRM.sample_points(j, :);
                        x = [point1(1); point2(1)];
                        y = [point1(2); point2(2)];
                        z = [point1(3); point2(3)];
                        line(x, y, z, 'Color', 'red')
                        edges = edges + 1;
                    end
                end
            end
        end

        function PRM = find_path(PRM, start, goal)
            % Find a path between a given start and goal
            if ~in_freespace(PRM.space, start) || ~in_freespace(PRM.space, ...
                    goal)
                error("Obscured start or goal")
            end
            to_add = [start; goal];
            for i = 1:2
                PRM.sample_points(end + 1, :) = to_add(i);
                kdt = KDTreeSearcher(PRM.sample_points, ...
                    'BucketSize', PRM.K + 1);
                neighbors = knnsearch(kdt, to_add(i, :), ...
                            'K', PRM.K + 1);
                added = 0;
                j = 0;
                while added < PRM.K
                    j = j + 1;
                    if j > PRM.K
                        neighbors = knnsearch(kdt, to_add(i, :), ...
                            'K', PRM.K + 1 + j);
                    end
                    if connects(PRM.space, to_add(i), PRM.sample_points(neighbors(j), :))
                        PRM.adjmat(PRM.sample_n + i, neighbors(j)) = 1;
                        added = added + 1;
                    end
                end
            end
            % Begin Astar
            onDeck = ODQueue();
            enqueue(onDeck, [PRM.sample_n + 1, norm(start - goal)])
            % Keep track of all done nodes
            done = [];
            seen = [];
            parents = zeros(PRM.sample_n);
            while true
                % Dequeue the next node from the queue
                state = dequeue(onDeck);
                % Find all neighbors of the state
                neighbors = zeros(PRM.K);
                found = 0;
                i = 1;
                while found < PRM.K
                    % If the entry in the adjmat is non-zero, i is a
                    % neighbor
                    if PRM.adjmat(state(1), i) ~= 0
                        neighbors(found + 1) = i;
                    end
                end
                % For each of the children
                for j = 1:length(neighbors)
                    neighbor_cost = PRM.adjmat(state(1), neighbors(j));
                    cost_togo = norm(PRM.sample_points(neighbors(j), :) - goal);
                    cost = neighbor_cost + cost_togo;
                    if ~ismember(neighbors(j), seen)
                        seen(end + 1) = neighbor(j);
                        parents(neighbor(j)) = state(1);
                        enqueue(onDeck, neighbors(j));
                    elseif ismember(neighbors(j), onDeck.list(:, 1))
                        index = find(onDeck.list(:, 1) == neighbors(j));
                        if cost < onDeck.list(index, 2)
                            parents(neighbor(j)) = state(1);
                            onDeck.list(index, :) = [neighbors(j), cost];
                            onDeck = sort(onDeck);
                        end
                    end
                end
                done(end + 1) = state;
                if PRM.sample_points(state(1)) == PRM.sample_points(end)
                    break
                end
                if isempty(onDeck.list)
                    error("~gunty")
                end
            end
            new_path = [];
            % Goal stored at last index of sample points
            l = length(PRM.sample_points);
            % Follow parents from goal to start
            while l ~= length(PRM.sample_points) - 1
                new_path(end + 1) = l;
                l = parents(l);
            end
            new_path(end + 1) = l;
            PRM.path = flip(new_path);
            disp(PRM.path)
        end
    end
end
