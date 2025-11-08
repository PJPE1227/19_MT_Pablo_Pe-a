%function used to make sure that every solution is inside the solution
%space
function Population = PopulationScaling(Population, x, y)
    % Number of dimensions per bat (each 2 entries = one point in 2D)
    dim = size(Population, 2);

    for d = 1:2:dim  % X coordinates (odd columns)
        Population(Population(:,d) < x(1), d) = x(1);
        Population(Population(:,d) > x(2), d) = x(2);
    end

    for d = 2:2:dim  % Y coordinates (even columns)
        Population(Population(:,d) < y(1), d) = y(1);
        Population(Population(:,d) > y(2), d) = y(2);
    end
end
