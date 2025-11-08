%function used to create APFs maps according to Kim
function z = potentialFieldKim(q, goal, obstacles, c_g, l_g, c_o, l_o)
    % Potencial de atracción (Kim, Wang, Shin)
    d_goal = norm(q - goal);
    Uatt = c_g * (1 - exp(-(d_goal^2) / l_g^2));
    
    % Potencial total inicial
    Utotal = Uatt;
    
    % Potenciales de repulsión para cada obstáculo
    for i = 1:length(obstacles)
        d_obs = distanceToObstacle(q, obstacles{i});
        Urep = c_o * exp(-(d_obs^2) / l_o^2);
        Utotal = Utotal + Urep;
    end
    
    z = Utotal;
end

function d = distanceToObstacle(q, obs)
    % Distancia mínima desde q al rectángulo obs
    closest_x = min(max(q(1), obs.x), obs.x + obs.w);
    closest_y = min(max(q(2), obs.y), obs.y + obs.h);
    d = norm([q(1) - closest_x, q(2) - closest_y]);
end
