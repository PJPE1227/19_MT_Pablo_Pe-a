%clear
%clc
function [t, movements, Best, AverageSolution, LastValueOf_N, trajectoryFlies, follower] = Firefly_Algorithm_APF(PopulationSize, NumMovements, Alpha, Beta0, Gamma, Theta, Dimension, optimiser, graphs, checkRadius, start, goal, method)
    %% Firefly Algorithm
    %PopulationSize    % Population size (Number of bats)
    %NumMovements      % Number of iterations
    %Alpha             % Parameter Alpha
    %Beta0             % Parameter Beta0
    %Gamma             % Parameter Gamma
    %Theta             % Parameter Theta
    %Dimension         % Number of dimensions
    %optimiser         % Optimisation function, check
    %optimisation_functions.m to know what functions are available
    %graphs            % 0 for no graphs, 1 for graphs and 2 to generate a
    %gif of the simulation
    %checkRadius       % Error radius allowed
    %start             % Start point for APFs test
    %goal              % Goal point fo APFs test
    %method            % choset or kim method to generate APFs

    %% Select Optimiser Function
    [x_range, y_range] = optimiser_ranges(optimiser);

    %% Initiate Firefly Population
    Population= zeros(PopulationSize, 2);
    Fitness = zeros(PopulationSize, 1);
    trajectoryFlies = zeros(NumMovements, PopulationSize*2);
    follower = 1;
    Population(1,1) = start(1);
    Population(1,2) = start(2);
    Fitness(1,1) = optimisation_functions(Population(1,1), Population(1,2), optimiser, "Optimiser", goal, method);
    for i = 2:PopulationSize
       Population(i,1) = (x_range(2)-x_range(1)).*rand(1,1)+x_range(1);
       Population(i,2) = (y_range(2)-y_range(1)).*rand(1,1)+y_range(1);
       Fitness(i,1) = optimisation_functions(Population(i,1), Population(i,2), optimiser, "Optimiser", goal, method);
    end

    %% Plot The Firefly Locations
    if graphs == 1 || graphs == 2
        points = PopulationPoints(Population, optimiser, [], goal, method);
    end
    if graphs == 2
        gif_filename = 'firefly_algorithm_APF.gif';  % name for the GIf
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
       Alpha = Alpha*Theta;                 % Reduce randomness by a factor of theta
       %Variable used to check i all the solutions are inside a designated
       %radius from the solution
       N = 0;
       for i = 1:PopulationSize
          trajectoryFlies(movements, (i*2)-1) = Population(i, 1);
          trajectoryFlies(movements, i*2) = Population(i, 2);
          for j = 1:PopulationSize
             Fitness(i,1) = optimisation_functions(Population(i,1), Population(i,2), optimiser, "Optimiser", goal, method);

             if Fitness(i,1)>=Fitness(j,1) %Brigther/more attractive
                 r = norm(Population(i,:)-Population(j,:));
                 Beta = Beta0 * exp(-Gamma*r.^2); % Attractiveness
                 Steps = Alpha*(2*rand(1, Dimension)-1);
                 Position = Population(i,:) + Beta*(Population(j,:)-Population(i,:))+Steps;
                 Position = PopulationScaling(Position, x_range, y_range);
                 
                 % Updating the position vectos
                 [NewFitness, ~] = optimisation_functions(Position(1), Position(2), optimiser, "Optimiser", goal, method);
                 
                 if NewFitness < Fitness(i,1)
                     Population(i,:) = Position;
                     Fitness(i,1) = NewFitness;
                 end
             end
          end
          %Check for break condition
          checkError = breakCheck(Position(1), Position(2), optimiser, checkRadius, goal);
          N = N + checkError;
       end
       [optval, optind] = min(Fitness);
       
       AverageSolution(movements) = mean(Fitness);
       
       BestSol(movements) = optval;
       Bestx(movements,:) = Population(optind,:);
       
       % Rank fireflies by their light intensity/objectives
       [Fitness, Index] = sort(Fitness);
       follower = find(Index==follower);
       Population = Population(Index,:);
       
       trajectoryFliesTEMP = zeros(NumMovements, PopulationSize*2);
        for order = 1:PopulationSize
            originalFlyIndex = Index(order);  % índice original del bat
            trajectoryFliesTEMP(:, (order*2)-1) = trajectoryFlies(:, (originalFlyIndex*2)-1);  % X coords
            trajectoryFliesTEMP(:, order*2) = trajectoryFlies(:, originalFlyIndex*2);        % Y coords
        end
        trajectoryFlies = trajectoryFliesTEMP;
       
       Best = Fitness(1);
       if graphs == 1 || graphs == 2
            points = PopulationPoints(Population, optimiser, points);
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
        fprintf("Best Solution is: x = %.5f, y = %.5f with a fitness of %.5f \n%", Population(1, 1), Population(1, 2), Best);

        figure(2);clf;
        plot(BestSol, 'LineWidth', 2);
        xlabel('Iteration Number');
        ylabel('Fitness Value');
        title('Firefly: Convergence Vs Iteration');
        grid on

        figure(3);clf;
        plot(AverageSolution, 'LineWidth', 2);
        xlabel('Iteration Number');
        ylabel('Average Fitness Value');
        title('Firefly: Convergence Vs Iteration');
        grid on
    end
end