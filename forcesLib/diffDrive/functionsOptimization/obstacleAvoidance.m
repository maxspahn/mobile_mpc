function ineq = obstacleAvoidance(z, p)
    x = z(1:3);
    objects = p(7:end);
    nbObstacles = size(objects, 1)/4;
    h1 = 1; % height of the robot
    spheres = [x(1), x(2), h1/2, h1/2];
    ineq = [];
    for i=1:nbObstacles
        for j = 1:size(spheres, 1)
            ineq = [ineq; sqrt((spheres(j, 1) - objects(4 * (i - 1) + 1))^2 +...
                    (spheres(j, 2) - objects(4 * (i - 1) + 2))^2 +...
                    (spheres(j, 3) - objects(4 * (i - 1) + 3))^2) - objects(4 * (i - 1) + 4) - spheres(j, 4)];
        end
    end

end

