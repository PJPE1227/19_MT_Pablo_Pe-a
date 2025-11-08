% available maps for testing A-L
function [obstacles] = mapTest(mapName, graph)
xRange = [-10, 10];
yRange = [-10, 10];

if strcmp(mapName, 'Map A')
    obstacles = {
        struct('x', -3, 'y', -3, 'w', 1, 'h', 1)  % obstacle 1
        struct('x', 2, 'y', -3, 'w', 1, 'h', 1)   % obstacle 2
        struct('x', 2, 'y', 2, 'w', 1, 'h', 1)  % obstacle 3
        struct('x', -3, 'y', 2, 'w', 1, 'h', 1)    % obstacle 4
    };
elseif strcmp(mapName, 'Map B')
    obstacles = {
        struct('x', -0.5, 'y', -3, 'w', 1, 'h', 1)  % obstacle 1
        struct('x', -5.5, 'y', -3, 'w', 1, 'h', 1)   % obstacle 2
        struct('x', -0.5, 'y', 2, 'w', 1, 'h', 1)  % obstacle 3
        struct('x', 4.5, 'y', 2, 'w', 1, 'h', 1)    % obstacle 4
    };
elseif strcmp(mapName, 'Map C')
    obstacles = {
        struct('x', -3, 'y', -3, 'w', 1, 'h', 1)  % obstacle 1
        struct('x', 2, 'y', -3, 'w', 1, 'h', 1)   % obstacle 2
        struct('x', 2, 'y', 2, 'w', 1, 'h', 1)  % obstacle 3
        struct('x', -3, 'y', 2, 'w', 1, 'h', 1)    % obstacle 4
        struct('x', -5.5, 'y', -5.5, 'w', 1, 'h', 1)  % obstacle 5
        struct('x', 4.5, 'y', -5.5, 'w', 1, 'h', 1)   % obstacle 6
        struct('x', 4.5, 'y', 4.5, 'w', 1, 'h', 1)  % obstacle 7
        struct('x', -5.5, 'y', 4.5, 'w', 1, 'h', 1)    % obstacle 8
    };
elseif strcmp(mapName, 'Map D')
    obstacles = {
        struct('x', -5.5, 'y', -5.5, 'w', 1, 'h', 5.5)  % obstacle 1
        struct('x', -5.5, 'y', -5.5, 'w', 5.5, 'h', 1)   % obstacle 2
        struct('x', 0.0, 'y', 4.5, 'w', 5.5, 'h', 1)  % obstacle 3
        struct('x', 4.5, 'y', 0, 'w', 1, 'h', 5.5)    % obstacle 4
        struct('x', 2.5, 'y', -3.5, 'w', 1, 'h', 1)    % obstacle 5
        struct('x', -3.5, 'y', 2.5, 'w', 1, 'h', 1)    % obstacle 5
    };
elseif strcmp(mapName, 'Map E')
    obstacles = {
        struct('x', -5.5, 'y', -5.5, 'w', 1, 'h', 5.5)  % obstacle 1
        struct('x', -5.5, 'y', -5.5, 'w', 5.5, 'h', 1)   % obstacle 2
        struct('x', 0.0, 'y', 4.5, 'w', 5.5, 'h', 1)  % obstacle 3
        struct('x', 4.5, 'y', 0, 'w', 1, 'h', 5.5)    % obstacle 4
        struct('x', -8.5, 'y', 3.0, 'w', 1, 'h', 5.5)    % obstacle 5
        struct('x', -7.5, 'y', 7.5, 'w', 5.5, 'h', 1)    % obstacle 6
        struct('x', 3.0, 'y', -7.5, 'w', 5.5, 'h', 1)    % obstacle 7
        struct('x', 7.5, 'y', -7.5, 'w', 1, 'h', 5.5)    % obstacle 8
    };
elseif strcmp(mapName, 'Map F')
    obstacles = {
        struct('x', -6, 'y', -11, 'w', 2, 'h', 13)
        struct('x', 4, 'y', -2, 'w', 2, 'h', 13)
    };
elseif strcmp(mapName, 'Map G')
    obstacles = {
        struct('x', -7, 'y', -11, 'w', 2, 'h', 13)
        struct('x', -1, 'y', -2, 'w', 2, 'h', 13)
        struct('x', 5, 'y', -11, 'w', 2, 'h', 13)
    };
elseif strcmp(mapName, 'Map H')
    obstacles = {
        struct('x', -6.5, 'y', -11, 'w', 1, 'h', 13)
        struct('x', -2.5, 'y', -2, 'w', 1, 'h', 13)
        struct('x', 1.5, 'y', -11, 'w', 1, 'h', 13)
        struct('x', 5.5, 'y', -2, 'w', 1, 'h', 13)
    };
elseif strcmp(mapName, 'Map I')
    randVal1 = 2;
    randVal2 = 1;
    obstacles = {
        struct('x', -2.5+((rand(1)*randVal1)-randVal2), 'y', -2.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)  % obstacle 1
        struct('x', 1.5+((rand(1)*randVal1)-randVal2), 'y', -2.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)   % obstacle 2
        struct('x', 1.5+((rand(1)*randVal1)-randVal2), 'y', 1.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)  % obstacle 3
        struct('x', -2.5+((rand(1)*randVal1)-randVal2), 'y', 1.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)    % obstacle 4
        struct('x', -6.5+((rand(1)*randVal1)-randVal2), 'y', -6.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)  % obstacle 5
        struct('x', 5.5+((rand(1)*randVal1)-randVal2), 'y', -6.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)   % obstacle 6
        struct('x', 5.5+((rand(1)*randVal1)-randVal2), 'y', 5.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)  % obstacle 7
        struct('x', -6.5+((rand(1)*randVal1)-randVal2), 'y', 5.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)    % obstacle 8
        struct('x', -0.5+((rand(1)*randVal1)-randVal2), 'y', -6.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)  % obstacle 9
        struct('x', 5.5+((rand(1)*randVal1)-randVal2), 'y', -0.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)   % obstacle 10
        struct('x', -6.5+((rand(1)*randVal1)-randVal2), 'y', -0.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)  % obstacle 11
        struct('x', -0.5+((rand(1)*randVal1)-randVal2), 'y', 5.5+((rand(1)*randVal1)-randVal2), 'w', 1, 'h', 1)    % obstacle 12
    };
elseif strcmp(mapName, 'Map J')
    obstacles = {
        struct('x', -4, 'y', -3, 'w', 8, 'h', 6)
        struct('x', -10.25, 'y', 9, 'w', 20.5, 'h', 1.5)
        struct('x', -10.25, 'y', -10.5, 'w', 20.5, 'h', 1.5)
        struct('x', 9, 'y', -10.25, 'w', 1.5, 'h', 20.5)
    };
elseif strcmp(mapName, 'Map K')
    % Mapa irregular compuesto con varios obstáculos rectangulares combinados
    obstacles = {
        % Zona central irregular (forma tipo cruz)
        struct('x', -4, 'y', -3, 'w', 8, 'h', 2.5)
        struct('x', -1.5, 'y', -5.5, 'w', 3, 'h', 5.5)
        
        % Obstáculo superior con forma escalonada
        struct('x', -9, 'y', 6.5, 'w', 5, 'h', 2)
        struct('x', -4, 'y', 7.5, 'w', 4, 'h', 2)
        struct('x', 0, 'y', 8, 'w', 5, 'h', 1.5)
        
        % Obstáculo inferior asimétrico
        struct('x', -8, 'y', -8.5, 'w', 4, 'h', 2)
        struct('x', -4, 'y', -9.5, 'w', 5, 'h', 1.5)
        struct('x', 1, 'y', -8.5, 'w', 4, 'h', 2)
        
        % Bordes del mapa (paredes externas)
        struct('x', -10.25, 'y', 9, 'w', 20.5, 'h', 1.5)
        struct('x', -10.25, 'y', -10.5, 'w', 20.5, 'h', 1.5)
        struct('x', 9, 'y', -10.25, 'w', 1.5, 'h', 20.5)
        struct('x', -10.5, 'y', -10.25, 'w', 1.5, 'h', 20.5)
    };
elseif strcmp(mapName, 'Map L')
    % Mapa con obstáculos definidos como polígonos irregulares ampliados
    obstacles = {
        % Polígono 1: triángulo irregular más grande (zona superior izquierda)
        struct('x', [-9, -5, -10], 'y', [2, 7, 8])
        
        % Polígono 2: polígono irregular grande reemplazando la "L" central
        struct('x', [-4, 5, 6, 2, -1, -3], 'y', [3, 3, -2, -4, -2, 1])
        
        % Polígono 3: forma pentagonal irregular más amplia (zona superior derecha)
        struct('x', [4, 9, 10, 8, 5], 'y', [4, 4, 8, 9, 6])
        
        % Polígono 4: forma irregular inferior más extendida (zona inferior izquierda)
        struct('x', [-9, -6, -2, -2, -8], 'y', [-9, -6, -8, -10, -11])
        
        % Polígono 5 (nuevo): forma trapezoidal irregular (zona inferior derecha)
        struct('x', [3, 9, 8, 4.5], 'y', [-9, -8, -5, -6])
    };
end


if graph == 1
    figure(1);clf; hold on; axis equal;grid on;
    xlim(xRange); ylim(yRange);
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
end
