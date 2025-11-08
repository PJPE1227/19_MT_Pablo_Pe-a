%clear
%clc
function [t, trajectoryFlies, obstacles,  pathCost] = Firefly_Algorithm_subPaths(PopulationSize, NumMovements, Alpha, Beta0, Gamma, Theta, Dimension, graphs, start, goal, map, numSubPaths, cost)
    %% Firefly Algorithm
    %PopulationSize    % Population size (Number of bats)
    %NumMovements      % Number of iterations
    %Alpha             % Parameter Alpha
    %Beta0             % Parameter Beta0
    %Gamma             % Parameter Gamma
    %Theta             % Parameter Theta
    %Dimension         % Number of dimensions
    %graphs            % 1 to create GIF 0 for nothing
    %start             % start point for the path
    %goal              % goal potin for the path
    %map               % obstacles map "Map A-L"
    %numSubPaths       % number of sub-paths
    %cost              % 1 to use APFs and 0 for regular
    
    t = 0;
    x_range = [-10, 10];
    y_range = [-10, 10];
    startPointx = start(1);
    startPointy = start(2);
    goalPointx = goal(1);
    goalPointy = goal(2);
    numNodes = Dimension/2;
    subPaths= zeros(numSubPaths, (2+numNodes)*2);
    obstacles = mapTest(map, 0);
    
    if(abs(startPointx-goalPointx)>=abs(startPointy-goalPointy))
        divisionAxis = 0; % se divide por el eje x
    else
        divisionAxis = 1; % se divide por el eje y
    end
    firstCollision = 1;
    if divisionAxis == 0
        realFirstGoalx = startPointx+(goalPointx-startPointx)/numSubPaths;
        while firstCollision == 1
            realFirstGoaly = (rand(1)*20)-10;
            for o = 1:length(obstacles)
                if max(collision([realFirstGoalx-0.5, realFirstGoaly], [realFirstGoalx+0.5, realFirstGoaly], obstacles{o}),...
                    collision([realFirstGoalx, realFirstGoaly-0.5], [realFirstGoalx, realFirstGoaly+0.5], obstacles{o}))
                    firstCollision = 1;
                    break;
                else
                    firstCollision = 0;
                end
            end
        end
        if startPointx >= 0
            x_rangeFirst = [realFirstGoalx, 10];
        else
            x_rangeFirst = [-10, realFirstGoalx];
        end
        y_rangeFirst = [-9.5, 9.5];
        if (goalPointx-startPointx) > 0
           direction = 1;
        else
            direction = 0;
        end
    else
        realFirstGoaly = startPointy+(goalPointy-startPointy)/numSubPaths;
        while firstCollision == 1
            realFirstGoalx = (rand(1)*20)-10;
            for o = 1:length(obstacles)
                if max(collision([realFirstGoalx-0.5, realFirstGoaly], [realFirstGoalx+0.5, realFirstGoaly], obstacles{o}),...
                    collision([realFirstGoalx, realFirstGoaly-0.5], [realFirstGoalx, realFirstGoaly+0.5], obstacles{o}))
                    firstCollision = 1;
                    break;
                else
                    firstCollision = 0;
                end
            end
        end
        x_rangeFirst = [-9.5, 9.5];
        if startPointy >= 0
            y_rangeFirst = [realFirstGoaly, 10];
        else
            y_rangeFirst = [-10, realFirstGoaly];
        end
        if (goalPointy-startPointy) > 0
           direction = 1;
        else
            direction = 0;
        end
    end
    
    
    
    [t1, ~, ~, ~, firstSubPath, ~] = Firefly_Algorithm_Path(PopulationSize, NumMovements, Alpha, Beta0, Gamma, Theta, Dimension, x_rangeFirst, y_rangeFirst, start, [realFirstGoalx, realFirstGoaly], map, cost, obstacles);
    t = t + t1;
    
    subPaths(1,:) = firstSubPath;
    
    for i = 1:(numSubPaths-1)
        nStart = [subPaths(i, -1+(2+numNodes)*2), subPaths(i, 0+(2+numNodes)*2)];
        nCollision = 1;
        if divisionAxis == 0
            nGoalx = startPointx+((goalPointx-startPointx)/numSubPaths)*(i+1);
            while nCollision == 1
                nGoaly = (rand(1)*20)-10;
                for o = 1:length(obstacles)
                    if max(collision([nGoalx-0.5, nGoaly], [nGoalx+0.5, nGoaly], obstacles{o}),...
                            collision([nGoalx, nGoaly-0.5], [nGoalx, nGoaly+0.5], obstacles{o}))
                        nCollision = 1;
                        break;
                    else
                        nCollision = 0;
                    end
                end
            end
        else
            nGoaly = startPointy+((goalPointy-startPointy)/numSubPaths)*(i+1);
            while nCollision == 1
                nGoalx = (rand(1)*20)-10;
                for o = 1:length(obstacles)
                    if max(collision([nGoalx-0.5, nGoaly], [nGoalx+0.5, nGoaly], obstacles{o}),...
                            collision([nGoalx, nGoaly-0.5], [nGoalx, nGoaly+0.5], obstacles{o}))
                        nCollision = 1;
                        break;
                    else
                        nCollision = 0;
                    end
                end
            end
        end
        if i==(numSubPaths-1)
           nGoalx = goalPointx;
           nGoaly = goalPointy;
        end
        
        if direction == 1
            if divisionAxis == 0
                nXrange = [nStart(1) ,nGoalx]; 
                nYrange = [-9.5 ,9.5];
            else
                nXrange = [-9.5, 9.5];
                nYrange = [nStart(2), nGoaly];
            end
        else
            if divisionAxis == 0
                nXrange = [nGoalx ,nStart(1)]; 
                nYrange = [-9.5 ,9.5];
            else
                nXrange = [-9.5, 9.5];
                nYrange = [nGoaly, nStart(2)];
            end
        end
        
        [tn, ~, ~, ~, nSubPath, ~] = Firefly_Algorithm_Path(PopulationSize, NumMovements, Alpha, Beta0, Gamma, Theta, Dimension, nXrange, nYrange, nStart, [nGoalx, nGoaly], map, cost, obstacles);
        t = t + tn;
        subPaths(1+i,:) = nSubPath;
    end
    
    trajectoryFlies = subPaths;
    pathCost = getDistancePath(trajectoryFlies);
    figure(4); clf; 
    hold on;
    axis equal; grid on;
    xlabel('X'); ylabel('Y');
    title('Evolución de la trayectoria compuesta - Algoritmo del murciélago');

    % Dibujar obstáculos
    for i = 1:length(obstacles)
        r = obstacles{i};
        rectangle('Position', [r.x, r.y, r.w, r.h], 'FaceColor', [0.6 0.1 0.1]);
    end

    % Inicializar GIF si graphs == 2
    if graphs == 1
        gif_filename = 'bat_algorithm_subpaths.gif';
        if exist(gif_filename, 'file')
            delete(gif_filename);
        end
    end

    % Dibujar los subtrayectos progresivamente
    for i = 1:numSubPaths
        coords = reshape(subPaths(i,:), 2, []);  % 2 x N
        plot(coords(1,:), coords(2,:), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('Subtrayecto %d', i));
        xlim(x_range); ylim(y_range);
        drawnow;

        % Guardar frame en el GIF
        if graphs == 1
            frame = getframe(gcf);
            im = frame2im(frame);
            [A, map] = rgb2ind(im, 256);
            if i == 1
                imwrite(A, map, gif_filename, "gif", "LoopCount", Inf, "DelayTime", 0.5);
            else
                imwrite(A, map, gif_filename, "gif", "WriteMode", "append", "DelayTime", 0.5);
            end
        end
    end

    legend show;

end