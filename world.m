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
            surf(x, y, zeros(length(x)), 'FaceColor', [0.982 0.333 0.176])

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

        function world = add_wall(world, x_start, x_end, y_start, ...
                y_end, z_start, z_end, radius)

            num_balls_x = ceil(abs(x_start - x_end)/radius);
            num_balls_y = ceil(abs(y_start - y_end)/radius);
            num_balls_z = ceil(abs(z_start - z_end)/radius);
            x_spacing = linspace(radius + x_start, x_end - radius, num_balls_x);
            y_spacing = linspace(radius + y_start, y_end - radius, num_balls_y);
            z_spacing = linspace(radius + z_start, z_end - radius, num_balls_z);

            for i = 1 : num_balls_x
                for j = 1 : num_balls_y
                    for k = 1 : num_balls_z
                        x_center = x_spacing(i);
                        y_center = y_spacing(j);
                        z_center = z_spacing(k);
                        ball = Obstacle([x_center y_center z_center], ...
                            radius);
                        world.obs(end + 1) = ball ; 
                    end
                end 
            end
        end

        function world = add_holy_wall(world, x_start, x_end, y_start, ...
                y_end, z_start, z_end, radius, hole_center, hole_radius)

            num_balls_x = ceil(abs(x_start - x_end)/radius);
            num_balls_y = ceil(abs(y_start - y_end)/radius);
            num_balls_z = ceil(abs(z_start - z_end)/radius);
            x_spacing = linspace(radius + x_start, x_end - radius, num_balls_x);
            y_spacing = linspace(radius + y_start, y_end - radius, num_balls_y);
            z_spacing = linspace(radius + z_start, z_end - radius, num_balls_z);

            for i = 1 : num_balls_x
                for j = 1 : num_balls_y
                    for k = 1 : num_balls_z
                        x_center = x_spacing(i);
                        y_center = y_spacing(j);
                        z_center = z_spacing(k);
                        if ~((x_center >= hole_center(1) - hole_radius & ...
                                x_center <= hole_center + hole_radius) & ...
                                (z_center >= hole_center(3) - hole_radius & ...
                                z_center <= hole_center(3) + hole_radius))
                            ball = Obstacle([x_center y_center z_center], ...
                                radius);
                            world.obs(end + 1) = ball;
                        end
                    end
                end 
            end
        end

        function in_freespace = in_freespace(world, point, clearance)
            % Check if a point in the map is in freespace
            % (outside any obstacles)
            in_freespace = true;
            % Ensure point in bounds
            if (point(1) > world.xmax || point(2) > world.ymax || point(3) ...
                    > world.zmax || point(1) <= 0 || point(2) <= 0 || ...
                    point(3) <= 0 + clearance)
                in_freespace = false;
            else
                % Check to ensure no collisions with obstacles
                for i = 1:length(world.obs)
                    dist = norm(world.obs(i).center - point);
                    if dist <= world.obs(i).radius + clearance
                        in_freespace = false;
                    end
                end
            end
        end

        function connects = connects(world, point1, point2, clearance)
            % Determine if two points connect with respect to the world
            connects = true;
            v = point2 - point1;
            %v_norm = v / norm(v);
            for i=1:length(world.obs)
                Q = world.obs(i).center;
                %u = Q - point1;
                %proj = dot(u, v_norm) * v_norm;
                % w = u - proj;
                t = ((Q(1) - point1(1)) * (point2(1) - point1(1)) + ...
                    (Q(2) - point1(2)) * (point2(2) - point1(2)) + ...
                    (Q(3) - point1(3)) * (point2(3) - point1(3))) ...
                    / ((norm(v))^2);
                if t > 1
                    t = 1;
                elseif t < 0
                    t = 0;
                end
                P = [(point1(1) + t * (point2(1) - point1(1))) ...
                    (point1(2) + t * (point2(2) - point1(2))) ...
                    (point1(3) + t * (point2(3) - point1(3)))];
                dist = norm(Q - P);
                if dist <= world.obs(i).radius + clearance
                    connects = false;
                    break
                end
            end
        end
    end
end