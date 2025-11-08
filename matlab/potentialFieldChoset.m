%function used to create APFs maps according to Choset
function z = potentialFieldChoset(q, goal, obstacles, k_att, d_star, k_rep, Q_star)
    % Potencial atractivo
    Utotal = attractivePotential(q, goal, k_att, d_star);
    
    % Potenciales repulsivos
    for i = 1:length(obstacles)
        Utotal = Utotal + repulsivePotential(q, obstacles{i}, k_rep, Q_star);
    end
    z = Utotal;
end

function Uatt = attractivePotential(q, goal, k_att, d_star)
    d = norm(q - goal);
    if d <= d_star
        Uatt = 0.5 * k_att * d^2;
    else
        Uatt = k_att * d_star * d - 0.5 * k_att * d_star^2;
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
