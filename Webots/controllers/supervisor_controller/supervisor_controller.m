% Código para el supervisor, se encarga de estar leyendo 
% constantemente el archivo de obstáculos y re-ubicando
% físicamente los mismos dentro de la simulación
function supervisor_controller

TIME_STEP = 64;

% Se obtienen los nodos de los obstáculos
obs_nodes(1) = wb_supervisor_node_get_from_def('obs1');
obs_nodes(2) = wb_supervisor_node_get_from_def('obs2');
obs_nodes(3) = wb_supervisor_node_get_from_def('obs3');
obs_nodes(4) = wb_supervisor_node_get_from_def('obs4');
obs_nodes(5) = wb_supervisor_node_get_from_def('obs5');
obs_nodes(6) = wb_supervisor_node_get_from_def('obs6');
obs_nodes(7) = wb_supervisor_node_get_from_def('obs7');
obs_nodes(8) = wb_supervisor_node_get_from_def('obs8');
obs_nodes(9) = wb_supervisor_node_get_from_def('obs9');
obs_nodes(10) = wb_supervisor_node_get_from_def('obs10');
obs_nodes(11) = wb_supervisor_node_get_from_def('obs11');
obs_nodes(12) = wb_supervisor_node_get_from_def('obs12');


% Se carga la posición actual de los obstáculos definidos
load('../obstacles.mat');
% se obtiene nodo del robot
robot_node = wb_supervisor_node_get_from_def('Pololu1');

t_field = wb_supervisor_node_get_field(robot_node, 'translation');

% Set the position (y is height)
wb_supervisor_field_set_sf_vec3f(t_field, [pos_ini(1), pos_ini(2), 0.05]); 

% Ciclo de simulación
while wb_robot_step(TIME_STEP) ~= -1
  load('../obstacles.mat');
  % Se re-ubican los obstáculos según su posición actual
  for i = 1:length(obs)
    translation_field = wb_supervisor_node_get_field(obs_nodes(i), 'translation');
    wb_supervisor_field_set_sf_vec3f(translation_field, [obs(i,:), 0.05]);
  end
  
  % Para los obstáculos no definidos, se mueven a una posición
  % en donde no sean visibles
  for j = length(obs)+1:12
    translation_field = wb_supervisor_node_get_field(obs_nodes(j), 'translation');
    wb_supervisor_field_set_sf_vec3f(translation_field, [0, 0, -0.1]);
  end
  
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end
