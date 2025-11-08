function pololu3pi_controller
rng('shuffle');
% Para poder emplear las funciones del Robotat (de ser necesario)
addpath('robotat'); 

TIME_STEP = 64; % tiempo de simulación en ms

% Dimensiones del robot
wheel_radius = 32; % mm
wheel_distance = 96 - 2*6.8; % mm

% Se obtienen los device tags/handles
right_motor = wb_robot_get_device('motor_1');
left_motor = wb_robot_get_device('motor_2');
compass = wb_robot_get_device('compass');
gps = wb_robot_get_device('gps');

% Se configuran y activan el GPS y la brújula
wb_gps_enable(gps, 10);
wb_compass_enable(compass, 10);

% Velocidad máxima, en rpm, de cada una de las ruedas. Se coloca de 
% esta manera ya que si bien la velocidad máxima admisible por los
% motores es de 800 rpm, por seguridad esta se limitará a la mitad. 
v_max = 800 / 2;

% Variables para las velocidades de las ruedas (en rpm <- OJO)
v_left = 0;
v_right = 10;

% Funciones de conversión entre rpm y rad/s para cambiar entre las
% dimensionales del robot y el simulador
rads2rpm = @(x) x * ( 60 / (2*pi));
rpm2rads = @(x) x * ( 2*pi / 60);

% Función de saturación para garantizar que las velocidades se
% encuentren en el rango adecuado
velsat = @(vel) sign(vel) * min(abs(vel), v_max);

% Se inicializan los motores
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0.0);
wb_motor_set_velocity(right_motor, 0.0);

% -----------------------------------------------------------------
% DEFINICIÓN DE VARIABLES E INICIALIZACIÓN DEL CONTROLADOR
% -----------------------------------------------------------------
% PID posición
kpP = 10;
kiP = 0.0001; 
kdP = 0.5;
EP = 0;
eP_1 = 0;

% PID orientación
kpO = 0.5;
kiO = 0; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
v0 = 500;
alpha = 10;

xi0 = [0; 0; 0];
u0 = [1; 1];
xi = xi0;
u = u0; % vector de entradas

eP = 10;
idx_path = 1;
% -----------------------------------------------------------------
% DEFINICIÓN DE OBSTÁCULOS Y CREACIÓN DEL MAPA
% -----------------------------------------------------------------

map = "Map I";    % mapa de obstaculos A-L 
subs = 0;         % 1 para usar subtrayectorias 0 para no
algo = 1;         % 1 para bat 2 para firefly
numSubPaths = 2;  % cantidad de subtrayectorias
cost = 0;         % 1 para usar APFs en el costo 0 para no

start = [-9, -4]; % punto de inicio de la trayectoria
goal = [9, 4];    % meta para la trayectoria

if algo == 1
  if subs == 0
    [t, ~, B, ~, track, obstacles] = Bat_Algorithm_Path(40, 200, 8, 0.7, 0.6, [-10, 10], [-10, 10], start, goal, map, cost);
  elseif subs == 1
    [t, trackBats, obstacles, B] = Bat_Algorithm_subPaths(40, 200, 8, 0.7, 0.6, 0, start, goal, map, numSubPaths, cost);
    track = reshape(trackBats.',1,[]);
  end
elseif algo == 2
  if subs == 0
    [t, ~, B, ~, track, obstacles] = Firefly_Algorithm_Path(40, 200, 0.6, 1.0, 0.01, 1.0, 8, [-10, 10], [-10, 10], start, goal, map, cost);
  elseif subs == 1
    [t, trackFlies, obstacles, B] = Firefly_Algorithm_subPaths(40, 200, 0.6, 1.0, 0.01, 1.0, 8, 0, start, goal, map, numSubPaths, cost);
    track = reshape(trackFlies.',1,[]);
  end
end
B
t
numObs = length(obstacles);

% inicializamos hasta obs12 como vacíos
obs = cell(1,12);
for i = 1:12
    obs{i} = [];
end

% asignamos centros reales
for i = 1:numObs
    r = obstacles{i};
    x_c = r.x + r.w/2;
    y_c = r.y + r.h/2;
    obs{i} = [x_c, y_c];
end
obs1 = obs{1};
obs2 = obs{2};
obs3 = obs{3};
obs4 = obs{4};
obs5 = obs{5};
obs6 = obs{6};
obs7 = obs{7};
obs8 = obs{8};
obs9 = obs{9};
obs10 = obs{10};
obs11 = obs{11};
obs12 = obs{12};
% Array de obstáculos
obs = [obs1; obs2; obs3; obs4; obs5; obs6; obs7; obs8; obs9; obs10; obs11; obs12];

% Se guarda la posición de los obstáculos (esto se hace para poder
% visualizar el cambio en la simulación)
save('../obstacles.mat', "obs");

% -----------------------------------------------------------------
% PLANIFICACIÓN DE MOVIMIENTO
% -----------------------------------------------------------------

pos_ini = [double(track(1)), double(track(2))];
save('../obstacles.mat', "pos_ini", '-append');

path_real = zeros((length(track)/2)-1, 2);
path_real_x = zeros((length(track)/2)-1, 1);
path_real_y = zeros((length(track)/2)-1, 1);
for i = 1:((length(track)/2))-1
   path_real_x(i) = track((i*2)+1);   
   path_real_y(i) = track((i*2)+2);
   path_real(i,1) = path_real_x(i);
   path_real(i,2) = path_real_y(i);
end
% --- Configuración de la gráfica de trayectoria ---
hold on;
grid on;
axis equal;
trajX = [];
trajY = [];
hPath = plot(nan, nan, 'k', 'LineWidth', 3.5); % línea de trayectoria
hRobot = plot(nan, nan, 'rhexagram', 'LineWidth', 4.0); % posición actual
counter = 0;
% Ciclo de simulación
while wb_robot_step(TIME_STEP) ~= -1

  % Se obtiene la posición y orientación actual del robot
  pos = wb_gps_get_values(gps);
  mag = wb_compass_get_values(compass);
  posx = pos(1)*1000; posy = pos(2)*1000;
  bearing = atan2d(mag(2), mag(1)) - 90;
  theta = atan2d(sind(-bearing), cosd(-bearing));
  
  % Ajuste del indice de la trayectoria
  idx_path = min(idx_path, (length(track)/2)-1);
  
  % coordenadas objetivo
  xg = path_real(idx_path, 1)*1000;
  yg = path_real(idx_path, 2)*1000;
    
  xi_i = [posx, posy, theta];
  
  e = [-xi_i(1) + xg; -xi_i(2) + yg];
  thetag = atan2d(e(2), e(1));
  
  eP = norm(e);
  eO = thetag - theta;
  eO = atan2d(sind(eO), cosd(eO));
  
  % Control de velocidad lineal
  kP = v0 * (1-exp(-alpha*eP^2)) / eP;
  v = kP * eP;
  
  % Control de velocidad angular
  eO_D = eO - eO_1;
  EO = EO + eO;
  w = kpO*eO + kiO*EO + kdO*eO_D;
  eO_1 = eO;
  
  % Verificacion de proximidad al objetivo
  if abs(eP) < 200.0
    idx_path = idx_path+1;
  end
  
  u = [v; w];
  
  if idx_path == (length(track)/2) && eP < 30.0
    v = 0;
    w = 0;
  end
  
  v_left = (v - (wheel_distance/2)*w)/wheel_radius;
  v_right = (v + (wheel_distance/2)*w)/wheel_radius;
  
  v_left = rads2rpm(v_left);
  v_right = rads2rpm(v_right);
  
  wb_motor_set_velocity(left_motor, -rpm2rads(velsat(v_left)));
  wb_motor_set_velocity(right_motor, -rpm2rads(velsat(v_right)));
  
  % --- Actualizar puntos de la trayectoria ---
  trajX(end+1) = posx/1000;
  trajY(end+1) = posy/1000;
  
  % Actualizar la línea y el punto del robot en el gráfico
  set(hPath, 'XData', trajX, 'YData', trajY);
  set(hRobot, 'XData', posx/1000, 'YData', posy/1000);

  % Flush para gráficos
  drawnow;

end

% cleanup code goes here: write data to files, etc.
