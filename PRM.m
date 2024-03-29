classdef PRM
    %  PRM - A simple naive PRM planner for 3D course planning
    
    properties
        sample_n double
        sample_points double
        K double
        space world
        adjmat double
        path double
        clearance double
        s_p_f
        pl_norm double
    end

    methods

        function PRM = PRM(sample_n, K, space, clearance)
           % Construct a PRM planner
           PRM.sample_n = sample_n;
           PRM.K = K;
           PRM.space = space;
           PRM.adjmat = zeros(PRM.sample_n);
           PRM.clearance = clearance;
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
                if in_freespace(PRM.space, sample, PRM.clearance)
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
                while found < PRM.K && j < PRM.sample_n
                    j = j + 1;
                    if j > PRM.K + extra
                        neighbors = knnsearch(kdt, PRM.sample_points(i, :), ...
                            'K', PRM.K + j + 1);
                    end
                    point1 = PRM.sample_points(neighbors(j), :);
                    point2 = PRM.sample_points(i, :);
                    if connects(PRM.space, point1, point2, PRM.clearance)
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
            for i=1:length(PRM.sample_points)
                edges = 0;
                j = 0;
                while edges < PRM.K && j < PRM.sample_n
                    j = j + 1;
                    if PRM.adjmat(i, j) ~= 0
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

        function PRM = adaptive_sampler(PRM, start, goal, std_dev)
            % Sample the space according to a normal distribution on the plane
            % which start and goal are on which passes through the least obstacles
            v = goal - start;
            v_norm = v / norm(v);
            sorted_v = sort(v_norm, 'descend');
            max_ind = find(v_norm == sorted_v(1));
            max_2_ind = find(v_norm == sorted_v(2));
            p = zeros(1, 3);
            ind_1 = max_ind(1);
            if max_2_ind(1) == ind_1
                ind_2 = max_2_ind(2);
            else
                ind_2 = max_2_ind(1);
            end
            p(ind_1) = v_norm(ind_2);
            p(ind_2) = -v_norm(ind_1);
            p = p / norm(p);

            % b = binormal vector, perpendicular to p and v_norm
            b = cross(v_norm, p);

            num_angles = 50;
            angles = linspace(0, pi, num_angles);

            normals = zeros(num_angles, 3);
            num_collisions = zeros(50, 1);

            for i = 1:length(angles)
                rotMat = [cos(angles(i)) -sin(angles(i)) 0;
                          sin(angles(i)) cos(angles(i)) 0;
                          0 0 1];
                % Normal vector to plane
                N = b * rotMat;
                normals(i, :) = N;
                coll = 0;
                for j = 1:length(PRM.space.obs)
                    % Checking for collision between obstacle and chosen plane 
                    obs_center = PRM.space.obs(j).center;
                    dist = dot((obs_center - start), N);
                    if dist < PRM.space.obs(j).radius
                        coll = coll + 1;
                    end
                end
                num_collisions(i) = coll;
            end
            % Find the index of the normal with the fewest collisions
            [~, I] = min(num_collisions);
            ind = I(1);
            N = normals(ind, :);
            PRM.pl_norm = N;

            % Show plane 
            show_plane(PRM, start, N)

            % Defining size of plane to uniformly sample by max distance to space limits 
            center = ((goal - start) / 2) + start;
            xmax = PRM.space.xmax;
            ymax = PRM.space.ymax;
            zmax = PRM.space.zmax;
            sz = sqrt(xmax^2 + ymax^2 + zmax^2);

            % Generating uniform sample on chosen plane 
            Q = null(N);
            while length(PRM.sample_points) < PRM.sample_n
                % Get random point in plane
                s_in_plane = center' + Q*((rand(2, 1)-0.5)*sz);
                % Shift point "S" along normal to plane by normally distributed 
                % distance to get final sample point
                s = s_in_plane' + N * normrnd(0, std_dev);
                if in_freespace(PRM.space, s, PRM.clearance)
                    PRM.sample_points(end + 1, :) = s;
                end
            end
        end

        function show_plane(PRM, point, normal)
            plot_world(PRM.space)
            % Assuming forms: point = [x,y,z] and normal = [x,y,z]
            d = -point*normal';
            [xx,yy]=ndgrid(1:PRM.space.xmax, 1:PRM.space.ymax);
            % Calculate corresponding z
            z = (-normal(1)*xx - normal(2)*yy - d)/normal(3);
            surf(xx,yy,z)
        end 

        function PRM = find_path(PRM, start, goal)
            % Find a path between a given start and goal
            if ~in_freespace(PRM.space, start, PRM.clearance) || ...
                ~in_freespace(PRM.space, goal, PRM.clearance)
                error("Obscured start or goal")
            end
            to_add = [start; goal];
            for i = 1:2
                PRM.sample_points(end + 1, :) = to_add(i, :);
                kdt = KDTreeSearcher(PRM.sample_points, ...
                    'BucketSize', PRM.K + 1);
                neighbors = knnsearch(kdt, to_add(i, :), ...
                            'K', PRM.K + 1);
                added = 0;
                j = 0;
                while added < PRM.K && j < PRM.sample_n - 1
                    j = j + 1;
                    if j > PRM.K
                        neighbors = knnsearch(kdt, to_add(i, :), ...
                            'K', PRM.K + 1 + j);
                    end
                    if connects(PRM.space, to_add(i, :), PRM.sample_points(neighbors(j + 1), :), PRM.clearance)
                        point1 = PRM.sample_points(PRM.sample_n + i, :);
                        point2 = PRM.sample_points(neighbors(j + 1), :);
                        PRM.adjmat(PRM.sample_n + i, neighbors(j + 1)) = norm(point1 - point2);
                        PRM.adjmat(neighbors(j + 1), PRM.sample_n + i ) = norm(point1 - point2);
                        added = added + 1;
                    end
                end
            end
            % Begin Astar
            onDeck = ODQueue();
            onDeck = enqueue(onDeck, [PRM.sample_n + 1, norm(start - goal)]);
            % Keep track of all done nodes
            done = [];
            seen = PRM.sample_n + 1;
            parents = zeros(PRM.sample_n, 1);
            deqed = [];
            while true
                % Dequeue the next node from the queue
                [onDeck, state] = dequeue(onDeck);
                deqed(end + 1) = state(1);
                % Find all neighbors of the state
                neighbors = zeros(1, PRM.K);
                found = 0;
                i = 1;
                while i <= length(PRM.sample_points)
                    % If the entry in the adjmat is non-zero, i is a
                    % neighbor
                    if PRM.adjmat(state(1), i) ~= 0
                        neighbors(1, found + 1) = i;
                        found = found + 1;
                    end
                    i = i + 1;
                end
                % For each of the children
                for j = 1:length(neighbors)
                    if neighbors(1, j) == 0
                        continue
                    end
                    neighbor_cost = PRM.adjmat(state(1), neighbors(1, j));
                    cost_togo = norm(PRM.sample_points(neighbors(1, j)) - goal);
                    cost = neighbor_cost + cost_togo;
                    if ~ismember(neighbors(1, j), seen)
                        seen(end + 1) = neighbors(1, j);
                        parents(neighbors(1, j)) = state(1);
                        onDeck = enqueue(onDeck, [neighbors(1, j), cost]);
                        onDeck = sort(onDeck);
                    elseif ismember(neighbors(1, j), onDeck.list(:, 1))
                        index = find(onDeck.list(:, 1) == neighbors(1, j));
                        if cost < onDeck.list(index, 2)
                            parents(neighbors(1, j)) = state(1);
                            onDeck.list(index, :) = [neighbors(1, j), cost];
                            onDeck = sort(onDeck);
                        end
                    end
                end
                done(end + 1) = state(1);
                if state(1) == length(PRM.sample_points)
                    break
                end
                if isempty(onDeck.list)
                    error("couldn't find a path")
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
        end

        function show_path(PRM)
            %show_sample(PRM)
            plot_world(PRM.space)
            start = PRM.sample_points(PRM.sample_n + 1, :);
            scatter3(start(1), start(2), start(3), 'filled', 'red')
            for i = 1:length(PRM.path) - 1
                point1 = PRM.sample_points(PRM.path(i), :);
                point2 = PRM.sample_points(PRM.path(i + 1), :);
                x = [point1(1), point2(1)];
                y = [point1(2), point2(2)];
                z = [point1(3), point2(3)];
                scatter3(point2(1), point2(2), point2(3), 'filled', 'red')
                line(x, y, z, 'Color', 'blue')
            end
        end

        function PRM = post_process(PRM)
           % Remove nodes in the same path that can be skipped without
           % collisions
           check_again = true;
           while check_again == true
               check_again = false;
               i = 2;
               while i < length(PRM.path)
                   prev_node = PRM.sample_points(PRM.path(i-1), :);
                   next_node = PRM.sample_points(PRM.path(i+1), :);
                   if connects(PRM.space, prev_node, next_node, PRM.clearance)
                       % skipping a node 
                       PRM.path(i) = [];
                       check_again = true;
                   else
                       i = i + 1;
                   end 
               end 
           end 
        end

        function PRM = smooth_path(PRM)
            % Smooth out the corners in the post-processed path

            % If the path is a straight line it can't be smoothed
            if length(PRM.path) <= 2
                return
            end

            % First, take the midpoints of each segment in the path
            path_pts = PRM.sample_points(PRM.path, :);
            mids = zeros(length(PRM.path) + 1, 3);
            mids(1, :) = path_pts(1, :);
            for i=1:length(path_pts) - 1
                mids(i + 1, :) = path_pts(i, :) + (path_pts(i + 1, :) ...
                    - path_pts(i, :)) / 2;
            end
            mids(end, :) = path_pts(end, :);
            % Create the smoothed path function
            PRM.s_p_f = cscvn(mids');
        end

        function find_path_length(PRM)
            % Use spline breakpoints for integration param vals 
            t_values = PRM.s_p_f.breaks;
            
            derivative_path = fnder(PRM.s_p_f);
            integrand = @(t) sqrt(sum(fnval(derivative_path, t).^2, 1));
            
            % Numerical integration using Simpson's rule
            % Integrand function returns an array of values when 
            % evaluated at a single point
            path_length = integral(integrand, t_values(1),...
                t_values(end), 'ArrayValued', true);
            
            disp(['Path length from numerical integration: ', num2str(path_length)]);
        end

        function show_smooth(PRM)
            % Plot a smoothed path function for the given PRM
            if length(PRM.path) <= 2
                return
            end
            plot_world(PRM.space);
            find_path_length(PRM);
            fnplt(PRM.s_p_f);
        end
    end
end
