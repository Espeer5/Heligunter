classdef PRM
    %  PRM - A simple naive PRM planner for 3D course planning
    
    properties
        sample_n int
        sample_points double
        K int 
        space world
    end

    methods 

        function PRM = PRM(sample_n, K, space)
           % Construct a PRM planner
           PRM.sample_n = sample_n 
           PRM.K = K
           PRM.space = space
        end

        function sample_PRM = sampler(PRM, space)
            sample_points = []
            while length(sample_points) < PRM.sample_n: 
                x = randi(world.xmax)
                y = randi(world.ymax)
                z = randi(world.zmax)
                sample = [x, y, z]
                if in_freespace(space, sample):
                    sample_points(end+1) = sample 
                end
            end
            sample_PRM.sample_points = sample_points
        end

        function show_sample(PRM)
            show_world(PRM.space)
            scatter3(PRM.sample_points(1), PRM.sample_points(2), PRM.sample_points(3))
        end
    end
end
