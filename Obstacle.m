classdef Obstacle
    % Obstacles - A class representing an obstacle within a 3D world. All
    %             obstacles are spheres which add up to larger structures.

    properties
        center double % Centroid of the obstacle
        radius double % Radius of spherical obstacle
        xmesh  double % Mesh of the obstacle
        ymesh  double % Mesh of the obstacle
        zmesh  double  % Mesh of the obstacle
        sphere_res = 10 % Constant resolution of the sphere mesh
    end

    methods
        function obj = Obstacle(center, radius)
            obj.center = center;
            obj.radius = radius;
            [obj.xmesh, obj.ymesh, obj.zmesh] = sphere(obj.sphere_res);
        end

        function plot_obstacle(obj)
            % Plot the obstacle in 3D space
            surf(obj.xmesh * obj.radius + obj.center(1), ...
                 obj.ymesh * obj.radius + obj.center(2), ...
                 obj.zmesh * obj.radius + obj.center(3));
            colormap([.5 .5 .5]);
        end
    end
end