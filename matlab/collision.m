% auxiliar function used to check if there's a collision during path
% planning
function intersects = collision(p1, p2, obstacle)
    % Rectángulo
    rectCorners = [
        obstacle.x, obstacle.y;
        obstacle.x + obstacle.w, obstacle.y;
        obstacle.x + obstacle.w, obstacle.y + obstacle.h;
        obstacle.x, obstacle.y + obstacle.h
    ];

    % Lados del rectángulo
    rectEdges = [
        rectCorners(1,:); rectCorners(2,:);
        rectCorners(2,:); rectCorners(3,:);
        rectCorners(3,:); rectCorners(4,:);
        rectCorners(4,:); rectCorners(1,:)
    ];

    intersects = false;
    for i = 1:4
        if segmentsIntersect(p1, p2, rectEdges(2*i-1,:), rectEdges(2*i,:))
            intersects = true;
            return;
        end
    end
end

function intersect = segmentsIntersect(a1, a2, b1, b2)
    % Chequea si dos segmentos se cruzan
    d1 = direction(b1, b2, a1);
    d2 = direction(b1, b2, a2);
    d3 = direction(a1, a2, b1);
    d4 = direction(a1, a2, b2);

    intersect = (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ...
                 ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)));
end

function val = direction(p1, p2, p3)
    val = (p3(1)-p1(1))*(p2(2)-p1(2)) - (p2(1)-p1(1))*(p3(2)-p1(2));
end
