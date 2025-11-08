%clear
%clc
function [t, movements, Best, AverageSolution, trajectoryBats, obstacles] = Bat_Algorithm_Path(PopulationSize, NumMovements, Dimension, Gamma, Alpha, x_range, y_range, start, goal, map, cost, premade)
    %% Bat Algorithm
    %PopulationSize    % Population size (Number of bats)
    %NumMovements      % Number of iterations
    %Dimension         % Number of dimensions
    %Gamma             % Parameter Gamma
    %Alpha             % Parameter Alpha
    %x_range           % x_range for solution space
    %y_range           % y_range for solution space
    %start             % start point for the path
    %goal              % goal potin for the path
    %map               % obstacles map "Map A-L"
    %cost              % 1 to use APFs and 0 for regular
    %premade           % leave as 0
    FreqMin = 0;       % Minimum frequency
    FreqMax  = 2;      % Maximum frequency

    %% Select Optimiser Function    
    switch nargin
        case 11
            obstacles = mapTest(map, 0);
        case 12
            obstacles = premade;
    end

    %% Initiate Bat Population
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

    [Best, ind] = min(Fitness);

    %% Initialising The Bat Dynamics
    BatLoudness = ones(PopulationSize, 1);
    for i = 1:PopulationSize
       BatLoudness(i) = rand + 1; 
    end
    BatPulseRate = zeros(PopulationSize, 1);
    for i = 1:PopulationSize
       BatPulseRate(i) = rand;
    end
    BatPulseRate0 = BatPulseRate;
    BatFrequency = zeros(PopulationSize, 1);
    BatVelocity = zeros(PopulationSize, (2+numNodes)*2);
    
    %% Run the simulation
    AverageSolution = zeros(NumMovements, 1);
    BestSol = zeros(NumMovements, 1);
    tic

    for movements = 1:NumMovements
        for bat = 1:PopulationSize
            %Update the dynamics
            BatFrequency(bat) = FreqMin + (FreqMax - FreqMin)*rand;
            BatVelocity(bat, 3:-2+(2+numNodes)*2) = BatVelocity(bat, 3:-2+(2+numNodes)*2) + ((Population(bat, 3:-2+(2+numNodes)*2) - Population(ind, 3:-2+(2+numNodes)*2))*BatFrequency(bat));
            Position = Population(bat, 3:-2+(2+numNodes)*2) + BatVelocity(bat, 3:-2+(2+numNodes)*2);

            %Perform a random walk
            if rand < BatPulseRate(bat)
                Position = Population(ind, 3:-2+(2+numNodes)*2)+(2*rand(1, Dimension)-1)*BatLoudness(bat);
            end

            %Scale the population
            Position = PopulationScaling(Position, x_range, y_range);

            %Update the fitness
            fullPath = [start(1), start(2), Position, goal(1), goal(2)];
            FitnessValue = tempPath(fullPath, obstacles, cost);
            if FitnessValue < Fitness(bat, 1) && rand < BatLoudness(bat)
                Fitness(bat, 1) = FitnessValue;
                Population(bat, 3:-2+(2+numNodes)*2) = Position;

                BatLoudness(bat) = BatLoudness(bat)*Alpha;
                BatPulseRate(bat) = BatPulseRate0(bat)*(1-exp(-Gamma*movements));
            end
            
            if FitnessValue < Best
                ind = bat;
                Best = FitnessValue;
            end
        end

        [optval, ~] = min(Fitness);
        
        AverageSolution(movements) = mean(Fitness);
        
        BestSol(movements) = optval;
        
        % Rank bats
        [Fitness, Index] = sort(Fitness);
        Population = Population(Index, :);
        
        ind = 1; Best = Fitness(1);
    end
    bestPath = Population(ind, :);  % Full path
    trajectoryBats = bestPath;
    coords = reshape(bestPath, 2, []);  % 2 x N
    figure(4);clf; plot(coords(1,:), coords(2,:), 'o-', 'LineWidth', 2);
    hold on;
    xlim(x_range); ylim(y_range);
    for i = 1:length(obstacles)
        r = obstacles{i};
        rectangle('Position', [r.x, r.y, r.w, r.h], 'FaceColor', [0.6 0.1 0.1]);
    end
    title('Mejor trayectoria encontrada por el algoritmo del murciélago');
    xlabel('X'); ylabel('Y');
    axis equal; grid on;

    t = toc;
end