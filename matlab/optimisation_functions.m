% function used to graph the maps
% you can graph the cost functions like this: optimisation_functions([], [], 'Beale', 'surf');
% and you can graph the obstacle maps like this: optimisation_functions([], [], 'APFmapK', 'surf', [9,9], 'Choset');
% don't use the last one with 'Map L'
function [z, min_val] = optimisation_functions(x, y, func, mode, goal, method)
    switch nargin
        case 4
            optimiser = str2func(func);

            if mode == "Optimiser"
                [z, min_val] = optimiser(x,y);
            else
                [x_range, y_range] = optimiser_ranges(func);
                x = linspace(x_range(1), x_range(2), 101);
                y = linspace(y_range(1), y_range(2), 101);
                z = zeros(101, 101);
                for j = 1:length(x)
                    for i = 1:length(y)
                        z(j, i) = optimiser(x(j), y(i));
                    end
                end

                figure(1);clf;
                xlabel('y')
                ylabel('x')
                if mode == "surf"
                    surf(y,x,z)
                    zlabel("fitness")
                elseif mode == "contour"
                    contour(y,x,z)
                end
            end
        case 6
            optimiser = str2func(func);

            if mode == "Optimiser"
                [z, min_val] = optimiser(x,y,goal,method);
            else
                [x_range, y_range] = optimiser_ranges(func);
                x = linspace(x_range(1), x_range(2), 101);
                y = linspace(y_range(1), y_range(2), 101);
                z = zeros(101, 101);
                for j = 1:length(x)
                    for i = 1:length(y)
                        z(j, i) = optimiser(x(j), y(i), goal,method);
                    end
                end

                figure(1);clf;
                xlabel('y')
                ylabel('x')
                if mode == "surf"
                    surf(y,x,z)
                    zlabel("fitness")
                elseif mode == "contour"
                    contour(y,x,z)
                end
            end
        otherwise
        
    end

end

function[z, min_val] = Beale(x, y)
    z = (1.5 - x + x*y)^2 + (2.25 - x + x*y^2)^2 + (2.625 - x + x*y^3)^2;
    min_val = 0;
end

function[z, min_val] = Ackley(x, y)
    z = -20*exp(-0.2*sqrt(0.5*(x^2 + y^2))) - exp(0.5*(cos(2*pi*x) + cos(2*pi*y))) + exp(1) + 20;
    min_val = 0;
end

function[z, min_val] = CrossTray(x, y)
    z = -0.0001 * (abs(sin(x) * sin(y) * exp(abs(100 - (sqrt(x^2 + y^2))/pi))) + 1)^0.1;
    min_val = -2.063;
end

function[z, min_val] = Sphere(x, y)
    z = x^2 + y^2;
    min_val = 0;
end

function[z, min_val] = Booth(x, y)
    z = (x + 2*y - 7)^2 + (2*x + y - 5)^2;
    min_val = 0;
end

function[z, min_val] = threeHump(x, y)
    z = 2*x^2 - 1.05*x^4 + (1/6)*x^6 + x*y + y^2;
    min_val = 0;
end

function[z, min_val] = Easom(x, y)
    z = -cos(x)*cos(y)*exp(-((x-pi)^2 + (y-pi)^2));
    min_val = -1;
end

function[z, min_val] = Michalewicz(y, x)
    m = 10;
    z = -((sin(x)*(sin(x^2/pi))^(2*m))+(sin(y)*(sin(2*y^2/pi))^(2*m)));
    min_val = -1.8013;
end

function[z, min_val] = Rosenbrock(x, y)
    z = 100*(y-x^2)^2 + (x-1)^2;
    min_val = 0;
end

function[z, min_val] = Eggcrate(x, y)
    z = x^2 + y^2 + 25*(sin(x)^2 + sin(y)^2);
    min_val = 0;
end

function [z, min_val] = APFmapA(x, y, goal, method)
    obstacles = mapTest('Map A', 0);
    if method == "Choset"
        z = potentialFieldChoset([x(1),y(1)], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([x(1), y(1)], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapB(x, y, goal, method)
    obstacles = mapTest('Map B', 0);
    if method == "Choset"
        z = potentialFieldChoset([x(1),y(1)], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([x(1), y(1)], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapC(x, y, goal, method)
    obstacles = mapTest('Map C', 0);
    if method == "Choset"
        z = potentialFieldChoset([x(1),y(1)], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([x(1), y(1)], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapD(x, y, goal, method)
    obstacles = mapTest('Map D', 0);
    if method == "Choset"
        z = potentialFieldChoset([x(1),y(1)], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([x(1), y(1)], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapE(x, y, goal, method)
    obstacles = mapTest('Map E', 0);
    if method == "Choset"
        z = potentialFieldChoset([x(1),y(1)], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([x(1), y(1)], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapF(x, y, goal, method)
    obstacles = mapTest('Map F', 0);
    if method == "Choset"
        z = potentialFieldChoset([y,x], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([y, x], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapG(x, y, goal, method)
    obstacles = mapTest('Map G', 0);
    if method == "Choset"
        z = potentialFieldChoset([y,x], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([y, x], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapH(x, y, goal, method)
    obstacles = mapTest('Map H', 0);
    if method == "Choset"
        z = potentialFieldChoset([y,x], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([y, x], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapI(x, y, goal, method)
    obstacles = mapTest('Map I', 0);
    if method == "Choset"
        z = potentialFieldChoset([y,x], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([y, x], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapJ(x, y, goal, method)
    obstacles = mapTest('Map J', 0);
    if method == "Choset"
        z = potentialFieldChoset([y,x], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([y, x], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapK(x, y, goal, method)
    obstacles = mapTest('Map K', 0);
    if method == "Choset"
        z = potentialFieldChoset([y,x], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([y, x], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end

function [z, min_val] = APFmapL(x, y, goal, method)
    obstacles = mapTest('Map L', 0);
    if method == "Choset"
        z = potentialFieldChoset([y,x], [goal(1), goal(2)], obstacles, 5, 2, 10, 5);
    else
        z = potentialFieldKim([y, x], [goal(1), goal(2)], obstacles, 500, 3, 500, 0.2);
    end
    min_val = 0;
end