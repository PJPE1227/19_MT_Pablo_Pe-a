%clear
%clc

function [t, movements, Best, besUncoveredArea, r, AverageSolution, trajectoryFlies] = Firefly_Algorithm_Sensors(PopulationSize, NumMovements, Alpha, Beta0, Gamma, Theta, Dimension, x_range, y_range, map, maxRad, minRad)
    %% Firefly Algorithm
    %PopulationSize    % Population size (Number of bats)
    %NumMovements      % Number of iterations
    %Alpha             % Parameter Alpha
    %Beta0             % Parameter Beta0
    %Gamma             % Parameter Gamma
    %Theta             % Parameter Theta
    %Dimension         % Number of dimensions
    %x_range           % x_range for solution space
    %y_range           % y_range for solution space
    %map               % obstacles map "Map A-L"
    %maxRad            % biggest radius for the sensor coverage
    %minRad            % smaller radius for the sensor converage, if maxRad
    %and minRad are the same then the radious won't be a variable

    %% Select Optimiser Function
    %optimiser = "Easom";
    %x_range = [-10, 10];
    %y_range = [-10, 10];
    
    obstacles = mapTest(map, 0);

    %% Initiate Firefly Population
    numNodes = Dimension/2;
    globRad = 0;
    if maxRad == minRad
       globRad = 1; 
    end
    
    if globRad == 0
        Population= zeros(PopulationSize, ((numNodes)*2)+1);
    else
        Population= zeros(PopulationSize, ((numNodes)*2));
    end

    Fitness = zeros(PopulationSize, 1);
    
    for i = 1:PopulationSize
       for j = 1:numNodes
          Population(i,(j*2)-1) = (x_range(2)-x_range(1)).*rand(1,1)+x_range(1);
          Population(i,(j*2)) = (y_range(2)-y_range(1)).*rand(1,1)+y_range(1);
       end
       if globRad == 0
           Population(i,end) = (maxRad-minRad)*rand + minRad; % radio global
           [~, Fitness(i,1)] = sensorCoverage(Population(i,:), obstacles, x_range, y_range);
       else
           [~, Fitness(i,1)] = sensorCoverage(Population(i,:), obstacles, x_range, y_range, maxRad);
       end
       
    end

    %% Plot The Firefly Locations
    %if graphs == 1
    %    points = PopulationPoints(Population, optimiser, []);
    %end
    %% Run the simulation
    AverageSolution = zeros(NumMovements, 1);
    BestSol = zeros(NumMovements, 1);
    tic

    for movements = 1:NumMovements
       Alpha = Alpha*Theta;                 % Reduce randomness by a factor of theta

       for i = 1:PopulationSize
          %trajectoryFlies(movements, (i*2)-1) = Population(i, 1);
          %trajectoryFlies(movements, i*2) = Population(i, 2);
          for j = 1:PopulationSize
             if globRad == 0
                [~,Fitness(i,1)] = sensorCoverage(Population(i,:), obstacles, x_range, y_range);
            else
                [~, Fitness(i,1)] = sensorCoverage(Population(i,:), obstacles, x_range, y_range, maxRad);
            end

             if Fitness(i,1)>=Fitness(j,1) %Brigther/more attractive
                 r = norm(Population(i,:)-Population(j,:));
                 Beta = Beta0 * exp(-Gamma*r.^2); % Attractiveness
                 if globRad == 0
                    Steps = Alpha*(2*rand(1, Dimension+1)-1);
                 else
                    Steps = Alpha*(2*rand(1, Dimension)-1);
                 end
                 Position = Population(i,:) + Beta*(Population(j,:)-Population(i,:))+Steps;
                 Position = PopulationScaling(Position, x_range, y_range);
                 
                 fullPath = Position;
                 % Updating the position vectos
                 if globRad == 0
                     [~,NewFitness] = sensorCoverage(fullPath, obstacles, x_range, y_range);
                 else
                     [~, NewFitness] = sensorCoverage(fullPath, obstacles, x_range, y_range, maxRad);
                 end
 
                 if NewFitness < Fitness(i,1)
                     Population(i,:) = Position;
                     Fitness(i,1) = NewFitness;
                 end
             end
          end
       end
       [optval, ~] = min(Fitness);
       
       AverageSolution(movements) = mean(Fitness);
       
       BestSol(movements) = optval;

       % Rank fireflies by their light intensity/objectives
       [Fitness, Index] = sort(Fitness);
       
       Population = Population(Index,:); 
       Best = Fitness(1);
       
    end
    bestPath = Population(1, :);  % Full path
    trajectoryFlies = bestPath;
    if globRad == 1
        coords = reshape(bestPath, 2, []);  % 2 x N
        r = maxRad;
        [besUncoveredArea, ~] = sensorCoverage(trajectoryFlies, obstacles, x_range, y_range, r);
    else
        coords = reshape(bestPath(1:numNodes*2),2,[]);
        r = bestPath((numNodes*2)+1);
        [besUncoveredArea, ~] = sensorCoverage(trajectoryFlies, obstacles, x_range, y_range);
    end

    %% Graficar
    figure(4); clf; hold on;
    xlim(x_range); ylim(y_range); axis equal; grid on;

    % Obstáculos
    for i = 1:length(obstacles)
        obs = obstacles{i};

        % Verificar si el obstáculo es poligonal o rectangular
        if numel(obs.x) > 1 && numel(obs.y) > 1
            % --- Obstáculo irregular definido por varios vértices ---
            patch(obs.x, obs.y, [0.6 0.1 0.1], 'EdgeColor', 'none');
        else
            % --- Obstáculo rectangular tradicional ---
            rectangle('Position', [obs.x obs.y obs.w obs.h], ...
                      'FaceColor', [0.6 0.1 0.1], 'EdgeColor', 'none');
        end
    end

    % Barrera del area de trabajo
    plot([-10,-10],[-10,10], 'g');
    plot([-10,10],[-10,-10], 'g');
    plot([10,10],[-10,10], 'g');
    plot([-10,10],[10,10], 'g');
    
    % Sensores y radio global
    for i = 1:size(coords,2)
        x = coords(1,i); y = coords(2,i);
        plot(x,y,'bo','MarkerFaceColor','b','MarkerSize',6);
        rectangle('Position',[x-r, y-r, 2*r, 2*r],'Curvature',[1 1], ...
                  'EdgeColor','b','LineStyle','--','FaceColor',[0 0 1 0.1]);
    end

    title('Cobertura de Sensores (Radio Global) por algoritmo de la luciernaga');
    xlabel('X'); ylabel('Y');
    
    t = toc;
end