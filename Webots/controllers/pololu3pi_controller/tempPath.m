%function used to calculate the cost of every path during the path planning
%tests
function [fitness]=tempPath(road, obstacles, cost)
    coords = reshape(road, 2, []);  % 2 x N matrix: each column is a point [x; y]
    diffs = diff(coords, 1, 2);     % differences between columns (segments)
    dists = sqrt(sum(diffs.^2, 1)); % Euclidean distances
    totalDist = sum(dists);
    
    collisionPenalty = 0;
    for s = 1:size(coords, 2)-1
        for o = 1:length(obstacles)
            if collision(coords(:,s), coords(:,s+1), obstacles{o})
                collisionPenalty = collisionPenalty + 1; % or higher
            end
        end
    end
    if cost == 1
        costPPoint = 0;
    %     Potenciales repulsivos
    %     for j = 1:length(coords)
    %         for i = 1:length(obstacles)
    %             costPPoint = costPPoint + repulsivePotential([coords(1,j),coords(2,j)], obstacles{i}, 10, 5);
    %         end
    %     end
        for s = 1:size(coords, 2)-1
            segment = [coords(:,s), coords(:,s+1)];
            for t = linspace(0, 1, 5)  % 5 muestras por segmento
                pos = segment(:,1) + t*(segment(:,2) - segment(:,1));
                for i = 1:length(obstacles)
                    costPPoint = costPPoint + repulsivePotential(pos', obstacles{i}, 10, 5);
                end
            end
        end
    end
    if cost == 1
        fitness = totalDist + 1e6 * collisionPenalty + 0.3*costPPoint;  % Large penalty to avoid obstacles
    else
        fitness = totalDist + 1e6 * collisionPenalty ;  % Large penalty to avoid obstacles
    end

end

function Urep = repulsivePotential(q, obs, k_rep, Q_star)
    d = distanceToRectangle(q, obs);
    if d <= Q_star
        Urep = 0.5 * k_rep * (1/d - 1/Q_star)^2;
    else
        Urep = 0;
    end
end

function dmin = distanceToRectangle(q, obs)
    dx = max([obs.x - q(1), 0, q(1) - (obs.x + obs.w)]);
    dy = max([obs.y - q(2), 0, q(2) - (obs.y + obs.h)]);
    dmin = sqrt(dx^2 + dy^2);
end

% function fitness = tempPath(road, obstacles)
%     % Convertir camino a coordenadas
%     coords = reshape(road, 2, []);  
%     diffs = diff(coords, 1, 2);     
%     dists = sqrt(sum(diffs.^2, 1)); 
%     totalDist = sum(dists);
% 
%     % Muestreo de segmentos (vectorizado)
%     samplesPerSegment = 5;  
%     tvals = linspace(0, 1, samplesPerSegment);
%     % Generar todos los puntos intermedios de todos los segmentos
%     segStarts = coords(:,1:end-1);
%     segEnds   = coords(:,2:end);
%     allPoints = [];
%     for ti = tvals
%         allPoints = [allPoints, segStarts + ti*(segEnds - segStarts)];
%     end
% 
%     % Calcular colisiones y potencial de una sola pasada
%     collisionPenalty = 0;
%     costPPoint = 0;
%     k_rep = 10;
%     Q_star = 5;
% 
%     for oi = 1:numel(obstacles)
%         obs = obstacles{oi};
%         
%         % Detectar colisiones (vectorizado)
%         inside = (allPoints(1,:) >= obs.x) & (allPoints(1,:) <= obs.x + obs.w) & ...
%                  (allPoints(2,:) >= obs.y) & (allPoints(2,:) <= obs.y + obs.h);
%         if any(inside)
%             collisionPenalty = collisionPenalty + sum(inside);
%         end
%         
%         % Calcular distancias mínimas a rectángulo para todos los puntos
%         dx = max([obs.x - allPoints(1,:); zeros(1,size(allPoints,2)); allPoints(1,:) - (obs.x + obs.w)], [], 1);
%         dy = max([obs.y - allPoints(2,:); zeros(1,size(allPoints,2)); allPoints(2,:) - (obs.y + obs.h)], [], 1);
%         dmin = sqrt(dx.^2 + dy.^2);
%         
%         % Potencial repulsivo (vectorizado)
%         mask = dmin <= Q_star;
%         costPPoint = costPPoint + sum(0.5 * k_rep * (1./dmin(mask) - 1/Q_star).^2);
%     end
% 
%     % Fitness final
%     fitness = totalDist + 2000 * collisionPenalty + costPPoint;
% end
