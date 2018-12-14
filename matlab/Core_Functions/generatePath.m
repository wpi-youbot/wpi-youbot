% *** function for path generation (returns [X Y] of path points)
function [path, vectors, vectors_len, path_len] = generatePath(start, target, obstacles)
    % defining the physical features of KUKA and obstacles
    kuka_safe_radius = 360;
    obstacle_safe_radius = 200;
    power = 2;

    Q = 1100000; % this is core parameter that changes the safe zone between obstacle and the robot
    target_force_scaling = 4;
    %target_force_scaling = 15;
    step = 2; % this step affects simulation position increment

    position = start;
    path = [];
    vectors = [];
    vectors_len = [];
    path_len = [];
    [num_obstacles, ~] = size(obstacles);

    iter = 0;
    while(norm(target - position) > 1.25*step*target_force_scaling)
        % add current position to the path
        path = [path; position];
        currentForces = [];

        iter = iter + 1;
        % Calculate interaction force between the object and each obstacle
        for it=1:num_obstacles
            % calculate the distance to obstacle
            distance = norm(obstacles(it,:) - position);
            if(distance < (kuka_safe_radius + obstacle_safe_radius))
                disp("ERROR! COLLISION WITH OBSTACLE!");
            end
            %distance

            % get the unit vector of force
            u_vec = (position - obstacles(it,:)) / distance;
            force_vec = u_vec * (Q/(distance^power));

            % Collecting the reaction force to all forces
            currentForces = [currentForces; force_vec];
        end

        % Calcualte interaction force from the target
        target_force_vec = target_force_scaling * (target - position) / norm(target - position);
        % Include target force together with the rest of the forces
        currentForces = [currentForces; target_force_vec];
        resultingForce = sum(currentForces);

        % Apply the effect of interaction: calculate new position
        positionIncrement = resultingForce * step;
        position = position + positionIncrement;

        vectors = [vectors; positionIncrement];
        %vectors_len = [vectors_len; norm(positionIncrement)];
    end

    iter; % show the number of iterations needed to complete the path

    vectors_len = vecnorm(vectors,2,2);
    path_len = cumsum(vectors_len);

end
