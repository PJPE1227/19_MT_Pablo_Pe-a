% function used to store the different map ranges
function [x,y] = optimiser_ranges(func)
    if func == "Beale"
        x = [-4.5, 4.5];
        y = [-4.5, 4.5];
    elseif func == "Ackley" || func == "Sphere" || func == "threeHump"
        x = [-5, 5];
        y = [-5, 5];
    elseif func == "CrossTray" || func == "Booth" || func == "APFmapA" || func == "APFmapB" || func == "APFmapC" || func == "APFmapD" || func == "APFmapE" || func == "APFmapF" || func == "APFmapG" || func == "APFmapH" || func == "APFmapI" || func == "APFmapJ" || func == "APFmapK" || func == "APFmapL"
        x = [-10, 10];
        y = [-10, 10];
    elseif func == "Easom"
        x = [0, 2*pi];
        y = [0, 2*pi];
    elseif func == "Michalewicz"
        x = [0, 4];
        y = [0, 4];
    elseif func == "Rosenbrock"
        x = [-2.048, 2.048];
        y = [-2.048, 2.048];
    elseif func == "Eggcrate"
        x = [-2*pi, 2*pi];
        y = [-2*pi, 2*pi];
    end
end
