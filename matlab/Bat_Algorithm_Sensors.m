%clear
%clc
function [t, movements, Best, besUncoveredArea, r, AverageSolution, trajectoryBats] = Bat_Algorithm_Sensors(PopulationSize, NumMovements, Dimension, Gamma, Alpha, x_range, y_range, map, maxRad, minRad)
    %% Bat Algorithm
    %PopulationSize    % Population size (Number of bats)
    %NumMovements      % Number of iterations
    %Dimension         % Number of dimensions
    %Gamma             % Parameter Gamma
    %Alpha             % Parameter Alpha
    %x_range           % x_range for solution space
    %y_range           % y_range for solution space
    %map               % obstacles map "Map A-L"
    %maxRad            % biggest radius for the sensor coverage
    %minRad            % smaller radius for the sensor converage, if maxRad
    %and minRad are the same then the radious won't be a variable
    FreqMin = 0;       % Minimum frequency
    FreqMax  = 2;      % Maximum frequency

    %% Select Optimiser Function
    obstacles = mapTest(map, 0);
    
    %% Initiate Bat Population
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
    if globRad == 1
        BatVelocity = zeros(PopulationSize, ((numNodes)*2));
    else
        BatVelocity = zeros(PopulationSize, ((numNodes)*2)+1);
    end
    
    %% Run the simulation
    AverageSolution = zeros(NumMovements, 1);
    BestSol = zeros(NumMovements, 1);  
    tic

    for movements = 1:NumMovements
        for bat = 1:PopulationSize
            %Update the dynamics
            BatFrequency(bat) = FreqMin + (FreqMax - FreqMin)*rand;
            BatVelocity(bat, :) = BatVelocity(bat, :) + ((Population(bat, :) - Population(ind, :))*BatFrequency(bat));
            Position = Population(bat, :) + BatVelocity(bat, :);

            %Perform a random walk
            if rand < BatPulseRate(bat)
                if globRad == 0
                    Position = Population(ind, :)+(2*rand(1, Dimension+1)-1)*BatLoudness(bat);
                else
                    Position = Population(ind, :)+(2*rand(1, Dimension)-1)*BatLoudness(bat);
                end
            end

            %Scale the population
            Position = PopulationScaling(Position, x_range, y_range);

            %Update the fitness
            fullPath = Position;
            if globRad == 0
                [~,FitnessValue] = sensorCoverage(fullPath, obstacles, x_range, y_range);
            else
                [~, FitnessValue] = sensorCoverage(fullPath, obstacles, x_range, y_range, maxRad);
            end
            
            if FitnessValue < Fitness(bat, 1) && rand < BatLoudness(bat)
                Fitness(bat, 1) = FitnessValue;
                Population(bat, :) = Position;

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
    bestSol = Population(ind, :);  
    trajectoryBats = bestSol;
    if globRad == 1
        coords = reshape(bestSol, 2, []);  % 2 x N
        r = maxRad;
        [besUncoveredArea, ~] = sensorCoverage(trajectoryBats, obstacles, x_range, y_range, r);
    else
        coords = reshape(bestSol(1:numNodes*2),2,[]);
        r = bestSol((numNodes*2)+1);
        [besUncoveredArea, ~] = sensorCoverage(trajectoryBats, obstacles, x_range, y_range);
    end
    

    %% Graficar
    figure(4); clf; hold on;
    xlim(x_range); ylim(y_range); axis equal; grid on;

    % Obstacles
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

    title('Cobertura de Sensores (Radio Global) por algoritmo del murcielago');
    xlabel('X'); ylabel('Y');

    t = toc;
end