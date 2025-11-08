%clear
%clc

function [t, movements, Best, AverageSolution, trajectoryFlies, obstacles] = Firefly_Algorithm_Path(PopulationSize, NumMovements, Alpha, Beta0, Gamma, Theta, Dimension, x_range, y_range, start, goal, map, cost, premade)
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
    %start             % start point for the path
    %goal              % goal potin for the path
    %map               % obstacles map "Map A-L"
    %cost              % 1 to use APFs and 0 for regular
    %premade           % leave as 0


    %% Select Optimiser Function
    switch nargin
        case 13
            obstacles = mapTest(map, 0);
        case 14
            obstacles = premade;
    end

    %% Initiate Firefly Population
    numNodes = Dimension/2;
    Population= zeros(PopulationSize, (2+numNodes)*2);
    startPointx = start(1);
    startPointy = start(2);
    Population(:,1) = startPointx;
    Population(:,2) = startPointy;
    goalPointx = goal(1);
    goalPointy = goal(2);
    Population(:,-1+(2+numNodes)*2) = goalPointx;
    Population(:,(2+numNodes)*2) = goalPointy;
    Fitness = zeros(PopulationSize, 1);
    
    for i = 1:PopulationSize
       for j = 1:numNodes
          Population(i,(j*2)+1) = (x_range(2)-x_range(1)).*rand(1,1)+x_range(1);
          Population(i,(j*2)+2) = (y_range(2)-y_range(1)).*rand(1,1)+y_range(1);
       end
       Fitness(i,1) = tempPath(Population(i,:), obstacles, cost);
    end

    %% Run the simulation
    AverageSolution = zeros(NumMovements, 1);
    BestSol = zeros(NumMovements, 1);
    tic

    for movements = 1:NumMovements
       Alpha = Alpha*Theta;                 % Reduce randomness by a factor of theta
       
       for i = 1:PopulationSize
          for j = 1:PopulationSize
             Fitness(i,1) = tempPath(Population(i,:), obstacles, cost);

             if Fitness(i,1)>=Fitness(j,1) %Brigther/more attractive
                 r = norm(Population(i,3:-2+(2+numNodes)*2)-Population(j,3:-2+(2+numNodes)*2));
                 Beta = Beta0 * exp(-Gamma*r.^2); % Attractiveness
                 Steps = Alpha*(2*rand(1, Dimension)-1);
                 Position = Population(i,3:-2+(2+numNodes)*2) + Beta*(Population(j,3:-2+(2+numNodes)*2)-Population(i,3:-2+(2+numNodes)*2))+Steps;
                 Position = PopulationScaling(Position, x_range, y_range);
                 
                 fullPath = [start(1), start(2), Position, goal(1), goal(2)];
                 % Updating the position vectos
                 NewFitness = tempPath(fullPath, obstacles, cost);
                 
                 if NewFitness < Fitness(i,1)
                     Population(i,3:-2+(2+numNodes)*2) = Position;
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
    coords = reshape(bestPath, 2, []);  % 2 x N
    figure(4);clf; plot(coords(1,:), coords(2,:), 'o-', 'LineWidth', 2);
    hold on;
    xlim(x_range); ylim(y_range);
    for i = 1:length(obstacles)
        r = obstacles{i};
        rectangle('Position', [r.x, r.y, r.w, r.h], 'FaceColor', [0.6 0.1 0.1]);
    end
    title('Mejor trayectoria encontrada por el algoritmo de la luciernaga');
    xlabel('X'); ylabel('Y');
    axis equal; grid on;
    
    t = toc;
end