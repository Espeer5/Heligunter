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
            hold off;
        end

        function world = add_arch(world, arch_center, arch_radius, n_spheres)
            % Add an arch obstacle to the world map
            theta_mesh = linspace(0, pi, n_spheres);
            for i = 1:n_spheres
                x = arch_center(1) + arch_radius * cos(theta_mesh(i));
                y = arch_center(2);
                z = arch_center(3) + arch_radius * sin(theta_mesh(i));
                disp([x y z])
                world.obs(end + 1) = Obstacle([x y z], 1);
            end
        end
    end
end