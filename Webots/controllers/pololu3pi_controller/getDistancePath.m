function distance = getDistancePath(road)
    road = reshape(road.',1,[]); 
    coords = reshape(road, 2, []);  % 2 x N matrix: each column is a point [x; y]
    diffs = diff(coords, 1, 2);     % differences between columns (segments)
    dists = sqrt(sum(diffs.^2, 1)); % Euclidean distances
    distance = sum(dists);
end