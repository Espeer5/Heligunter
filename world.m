classdef world
    % WORLD - This class represents a 3D map through which courses can be
    %         planned by the helicopter

    properties
        xmax double    % Maximum x coordinate
        ymax double    % Maximum y coordinate
        zmax double    % Maximum z coordinate
        obs  Obstacle  % Collection of obstacles in the world map
    end

    methods

        function world = world(xmax, ymax, zmax, obs)
            % Construct a world map with the given coordinate set and
            % obstacles
            world.xmax = xmax;
            world.ymax = ymax;
            world.zmax = zmax;
            % Pass in Obstacle.empty to initialize with no obstacles
            world.obs = obs; 
        end

        function plot_world(world)
            % Plot the world map with the obstacles
            figure;
            xlim([0 world.xmax]);
            ylim([0 world.ymax]);
            zlim([0 world.zmax]);
            hold on;
            for i = 1:length(world.obs)
                world.obs(i).plot_obstacle();
            end
            view(-30,30)
            % Add a floor
            [x, y] = meshgrid(0:world.xmax, 0:world.ymax);
            surf(x, y, zeros(length(x)), 'FaceColor', 'black')

        end

        function world = add_arch(world, arch_center, arch_radius, ...
                n_spheres, girth)
            % Add an arch obstacle to the world map
            theta_mesh = linspace(0, pi, n_spheres);
            for i = 1:n_spheres
                x = arch_center(1) + arch_radius * cos(theta_mesh(i));
                y = arch_center(2);
                z = arch_center(3) + arch_radius * sin(theta_mesh(i));
                world.obs(end + 1) = Obstacle([x y z], girth);
            end
        end

        function world = add_terrain(world, num_bumps, avg_height)
            % Add a randomly generated bumpy terrain to the world
            for i = 1:num_bumps
                bump = Obstacle([randi(world.xmax) randi(world.ymax) -1], ...
                    normrnd(avg_height, 1));
                world.obs(end + 1) = bump;
            end
        end

        function in_freespace = in_freespace(world, point)
            % Check if a point in the map is in freespace
            % (outside any obstacles)
            in_freespace = true;
            % Ensure point in bounds
            if (point(1) > world.xmax || point(2) > world.ymax || point(3) ...
                    > world.zmax || point(1) <= 0 || point(2) <= 0 || ...
                    point(3) <= 0)
                in_freespace = false;
            else
                % Check to ensure no collisions with obstacles
                for i = 1:length(world.obs)
                    dist = norm(world.obs(i).center - point);
                    if dist <= world.obs(i).radius
                        in_freespace = false;
                    end
                end
            end
        end

        function connects = connects(world, point1, point2)
            % Determine if two points connect with respect to the world
            connects = true;
            v = point2 - point1;
            v_norm = v / norm(v);
            for i=1:length(world.obs)
                Q = world.obs(i).center;
                u = Q - point1;
                proj = dot(u, v_norm) * v_norm;
                w = u - proj;
                if norm(w) <= world.obs(i).radius
                    connects = false;
                    break
                end
            end
        end
    end
end