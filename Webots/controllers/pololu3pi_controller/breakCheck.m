% auxiliar function used to check the error radius set on some of the
% functions
function [n] = breakCheck(x, y, func, checkRadius,goal)
    switch nargin
        case 4
            if func == "Beale"
                radius = sqrt((3-x)^2 + (0.5-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "Ackley"
                radius = sqrt((0-x)^2 + (0-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "CrossTray"
                radius1 = sqrt((1.34941-x)^2 + (-1.34941-y)^2);
                radius2 = sqrt((1.34941-x)^2 + (1.34941-y)^2);
                radius3 = sqrt((-1.34941-x)^2 + (1.34941-y)^2);
                radius4 = sqrt((-1.34941-x)^2 + (-1.34941-y)^2);
                if radius1 <= checkRadius || radius2 <= checkRadius || radius3 <= checkRadius || radius4 <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "Sphere"
                radius = sqrt((0-x)^2 + (0-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "Booth"
                radius = sqrt((1-x)^2 + (3-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "threeHump"
                radius = sqrt((0-x)^2 + (0-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "Easom"
                radius = sqrt((3.14159265-x)^2 + (3.14159265-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "Michalewicz"
                radius = sqrt((1.57-x)^2 + (2.2-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "Rosenbrock"
                radius = sqrt((1-x)^2 + (1-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            elseif func == "Eggcrate"
                radius = sqrt((0-x)^2 + (0-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            else
                n = 0;
            end
        case 5
            if func == "APFmapA" || func == "APFmapB" || func == "APFmapC" || func == "APFmapD" || func == "APFmapE" || func == "APFmapF" || func == "APFmapG" || func == "APFmapH" || func == "APFmapI"
                radius = sqrt((goal(1)-x)^2 + (goal(2)-y)^2);
                if radius <= checkRadius
                    n = 1;
                else
                    n = 0;
                end
            end
        otherwise
    end
end