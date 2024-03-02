classdef PRM
    %  PRM - A simple naive PRM planner for 3D course planning
    
    properties
        sample_n uint64
        sample_points double
        K uint64
        space world
    end

    methods 

        function PRM = PRM(sample_n, K, space)
           % Construct a PRM planner
           PRM.sample_n = sample_n;
           PRM.K = K;
           PRM.space = space;
        end

        function PRM = sampler(PRM)
            new_sample_points = zeros(PRM.sample_n, 3);
            index = 0;
            while length(new_sample_points) < PRM.sample_n
                index = index + 1;
                x = randi(PRM.space.xmax);
                y = randi(PRM.space.ymax);
                z = randi(PRM.space.zmax);
                sample = [x, y, z];
                if in_freespace(PRM.space, sample)
                    new_sample_points(index, 1:3) = sample;
                end
            end
            PRM.sample_points = new_sample_points;
        end

        function show_sample(PRM)
            plot_world(PRM.space)
            scatter3(PRM.sample_points(:, 1), PRM.sample_points(:, 2), ...
                PRM.sample_points(:, 3), "filled", "blue")
        end
    end
end
