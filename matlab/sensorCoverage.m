%function used to calculate cost of every solution of sensore coverage
function [uncoveredArea, fitness] = sensorCoverage(solution, obstacles, x_range, y_range, radius)
    %% ---- Extracción y validación ----
    n = numel(solution);
    uncoveredArea = 0;
    coordsLen = n - 1;

    switch nargin
        case 4
            numSensors = coordsLen/2;
            coords = reshape(solution(1:end-1), 2, []);  
            r = solution(end);
            if r <= 0
                fitness = inf;
                return;
            end
        case 5
            numSensors = n/2;
            coords = reshape(solution, 2, []);  
            r = radius;
    end

    %% ---- Parámetros ----
    xmin = x_range(1); xmax = x_range(2);
    ymin = y_range(1); ymax = y_range(2);
    nx = 50; ny = 50;
    w_out = 20.0; w_overlap = 20.0; w_center = 10.0;

    %% ---- Grilla ----
    xlin = linspace(xmin, xmax, nx);
    ylin = linspace(ymin, ymax, ny);
    [X, Y] = meshgrid(xlin, ylin);
    cellArea = (xlin(2)-xlin(1)) * (ylin(2)-ylin(1));

    %% ---- Máscara de obstáculos ----
    obstacleMask = false(size(X));
    for o = 1:length(obstacles)
        obs = obstacles{o};
        if isfield(obs, 'w') && isfield(obs, 'h')
            xmin_o = min(obs.x, obs.x + obs.w);
            xmax_o = max(obs.x, obs.x + obs.w);
            ymin_o = min(obs.y, obs.y + obs.h);
            ymax_o = max(obs.y, obs.y + obs.h);
            obstacleMask = obstacleMask | (X >= xmin_o & X <= xmax_o & Y >= ymin_o & Y <= ymax_o);
        elseif isfield(obs, 'x') && numel(obs.x) > 1 && isfield(obs, 'y')
            obstacleMask = obstacleMask | inpolygon(X, Y, obs.x, obs.y);
        end
    end
    mapMask = ~obstacleMask;
    mapArea = sum(mapMask(:)) * cellArea;

    %% ---- Cobertura vectorizada ----
    overlapCount = zeros(size(X));
    circleArea = pi * r^2;

    % Calcular cobertura de todos los sensores en bloque
    for i = 1:numSensors
        dx = X - coords(1,i);
        dy = Y - coords(2,i);
        inCircle = (dx.^2 + dy.^2) <= r^2;

        % Solo contar área libre
        overlapCount = overlapCount + double(inCircle & mapMask);
    end

    % Área cubierta válida
    coveredMask = overlapCount >= 1;
    coveredCells = sum(coveredMask(:));
    coveredArea = coveredCells * cellArea;

    % Área de solapamiento (celdas con más de un sensor)
    overlapCells = sum(overlapCount(:) > 1);
    overlapArea = overlapCells * cellArea;

    %% ---- Penalización por sensores fuera del mapa ----
    % Distancia de los centros al borde del mapa
    outsideLeft   = coords(1,:) - xmin < r;
    outsideRight  = xmax - coords(1,:) < r;
    outsideBottom = coords(2,:) - ymin < r;
    outsideTop    = ymax - coords(2,:) < r;
    areaOutsideTotal = sum(outsideLeft | outsideRight | outsideBottom | outsideTop) * (circleArea * 0.2);

    %% ---- Penalización por centro dentro de obstáculo (vectorizado) ----
    % Evalúa todos los centros a la vez con inpolygon o comparaciones lógicas
    inObs = false(1, numSensors);
    for o = 1:length(obstacles)
        obs = obstacles{o};
        if isfield(obs, 'w') && isfield(obs, 'h')
            xmin_o = min(obs.x, obs.x + obs.w);
            xmax_o = max(obs.x, obs.x + obs.w);
            ymin_o = min(obs.y, obs.y + obs.h);
            ymax_o = max(obs.y, obs.y + obs.h);
            inObs = inObs | (coords(1,:) >= xmin_o & coords(1,:) <= xmax_o & ...
                             coords(2,:) >= ymin_o & coords(2,:) <= ymax_o);
        elseif isfield(obs, 'x') && numel(obs.x) > 1 && isfield(obs, 'y')
            inObs = inObs | inpolygon(coords(1,:), coords(2,:), obs.x, obs.y);
        end
    end
    countCenterInObs = sum(inObs);

    %% ---- Fitness ----
    uncoveredArea = mapArea - coveredArea;
    penalty_out = w_out * areaOutsideTotal;
    penalty_overlap = w_overlap * overlapArea;
    penalty_center = w_center * countCenterInObs * mapArea;

    fitness = uncoveredArea + penalty_out + penalty_overlap + penalty_center;
end


