%function used to graph the points during the cost function testing
function points = PopulationPoints(Population, optimiser, points, goal, method)
    switch nargin
        case 3
            if isempty(points)
                figure(1)
                cla
                optimisation_functions([], [], optimiser, "contour");
            else
                reset(points);
            end
            hold on
            points = scatter(Population(:, 2), Population(:, 1), 'ko', 'MarkerFaceColor', 'k');
            drawnow()
        case 5
            if isempty(points)
                figure(1)
                cla
                optimisation_functions([], [], optimiser, "contour", goal, method);
            else
                reset(points);
            end
            hold on
            points = scatter(Population(:, 2), Population(:, 1), 'ko', 'MarkerFaceColor', 'k');
            drawnow()
        otherwise
    end
end