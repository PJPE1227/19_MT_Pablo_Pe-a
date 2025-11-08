%clear
%clc
function [t, movements, Best, AverageSolution, LastValueOf_N, trajectoryBats, follower] = Bat_Algorithm_APF(PopulationSize, NumMovements, Dimension, Gamma, Alpha, optimiser, graphs, checkRadius, start, goal, method)
    %% Bat Algorithm
    %PopulationSize    % Population size (Number of bats)
    %NumMovements      % Number of iterations
    %Dimension         % Number of dimensions
    %Gamma             % Parameter Gamma
    %Alpha             % Parameter Alpha
    %optimiser         % Optimisation function, check
    %optimisation_functions.m to know what functions are available
    %graphs            % 0 for no graphs, 1 for graphs and 2 to generate a
    %gif of the simulation
    %checkRadius       % Error radius allowed
    %start             % Start point for APFs test
    %goal              % Goal point fo APFs test
    %method            % choset or kim method to generate APFs
    FreqMin = 0;       % Minimum frequency
    FreqMax  = 2;      % Maximum frequency

    %% Select Optimiser Function
    [x_range, y_range] = optimiser_ranges(optimiser);

    %% Initiate Bat Population
    Population= zeros(PopulationSize, 2);
    Fitness = zeros(PopulationSize, 1);
    trajectoryBats = zeros(NumMovements, PopulationSize*2);
    follower = 1;
    Population(1,1) = start(1);
    Population(1,2) = start(2);
    Fitness(1,1) = optimisation_functions(Population(1,1), Population(1,2), optimiser, "Optimiser", goal, method);
    for i = 2:PopulationSize
       Population(i,1) = (x_range(2)-x_range(1)).*rand(1,1)+x_range(1);
       Population(i,2) = (y_range(2)-y_range(1)).*rand(1,1)+y_range(1);
       Fitness(i,1) = optimisation_functions(Population(i,1), Population(i,2), optimiser, "Optimiser", goal, method);
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
    %BatFrequency = zeros(PopulationSize, Dimension);
    BatVelocity = zeros(PopulationSize, Dimension);

    %% Plot The Bat Locations
    if graphs == 1 || graphs == 2
        points = PopulationPoints(Population, optimiser, [], goal, method);
    end
    if graphs == 2
        gif_filename = 'bat_algorithm_APF.gif';  % name for the GIf
        if exist(gif_filename, 'file')
            delete(gif_filename);  % delete GIF if it exists already
        end
    end
    %% Run the simulation
    AverageSolution = zeros(NumMovements, 1);
    BestSol = zeros(NumMovements, 1);
    Bestx = zeros(NumMovements, 2);
    LastValueOf_N = 0;
    
    tic

    for movements = 1:NumMovements
        %Variable used to check i all the solutions are inside a designated
        %radius from the solution
        N = 0;  
        for bat = 1:PopulationSize
            trajectoryBats(movements, (bat*2)-1) = Population(bat, 1);
            trajectoryBats(movements, bat*2) = Population(bat, 2);
            %Update the dynamics
            BatFrequency(bat) = FreqMin + (FreqMax - FreqMin)*rand;
            BatVelocity(bat, :) = BatVelocity(bat, :) + ((Population(bat, :) - Population(ind, :))*BatFrequency(bat));
            Position = Population(bat, :) + BatVelocity(bat, :);

            %Perform a random walk
            if rand < BatPulseRate(bat)
                Position = Population(ind, :)+(2*rand(1, Dimension)-1)*BatLoudness(bat);
            end

            %Scale the population
            Position = PopulationScaling(Position, x_range, y_range);

            %Update the fitness
            [FitnessValue, ~] = optimisation_functions(Position(1), Position(2), optimiser, "Optimiser", goal, method);
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

            %Check for break condition
            checkError = breakCheck(Population(bat, 1), Population(bat, 2), optimiser, checkRadius, goal);
            N = N + checkError;
        end

        [optval, optind] = min(Fitness);
        
        AverageSolution(movements) = mean(Fitness);
        
        BestSol(movements) = optval;
        Bestx(movements,:) = Population(optind,:);
        % Rank bats
        [Fitness, Index] = sort(Fitness);
        follower = find(Index==follower);
        Population = Population(Index, :);
       
        trajectoryBatsTEMP = zeros(NumMovements, PopulationSize*2);
        for order = 1:PopulationSize
            originalBatIndex = Index(order);  % Original index of bat
            trajectoryBatsTEMP(:, (order*2)-1) = trajectoryBats(:, (originalBatIndex*2)-1);  % X coords
            trajectoryBatsTEMP(:, order*2) = trajectoryBats(:, originalBatIndex*2);        % Y coords
        end
        trajectoryBats = trajectoryBatsTEMP;
        
        ind = 1; Best = Fitness(1);
        
        if graphs == 1 || graphs == 2
            points = PopulationPoints(Population, optimiser, points, goal, method);
        end
        if graphs == 2
            % Get actual frame
            frame = getframe(gcf);  % Get actual figure
            im = frame2im(frame);
            [A, map] = rgb2ind(im, 256);

            % Save in the GIf
            if movements == 1
                imwrite(A, map, gif_filename, "gif", "LoopCount", Inf, "DelayTime", 0.2);
            else
                imwrite(A, map, gif_filename, "gif", "WriteMode", "append", "DelayTime", 0.2);
            end
        end
        LastValueOf_N = N;
        if N == PopulationSize
           break
        end
    end
    
    t = toc;
    if graphs == 1
        fprintf("Simulation ended after %.4f seconds (Movements: %0.0f)\n", t, movements);
        fprintf("Best Solution is: x = %.5f, y = %.5f with a fitness of %.5f \n%", Population(ind, 1), Population(ind, 2), Best);
    
        figure(2);clf;
        plot(BestSol, 'LineWidth', 2);
        xlabel('Iteration Number');
        ylabel('Fitness Value');
        title('Bat: Convergence Vs Iteration');
        grid on

        figure(3);clf;
        plot(AverageSolution, 'LineWidth', 2);
        xlabel('Iteration Number');
        ylabel('Average Fitness Value');
        title('Bat: Convergence Vs Iteration');
        grid on
    end
end