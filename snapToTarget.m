function [location, overrun] = snapToTarget(location, target, theta)
    if theta >= 0 && theta < 90 
        if location(1) >= target(1) && location(2) >= target(2)
            overrun = sqrt(((target(1)-location(1))^2)+((target(2)-location(2))^2)); 
            location(:) = target(:);
        else
            overrun = 0;
        end
    elseif theta >= 90 && theta < 180
        if location(1) >= target(1) && location(2) <= target(2)
            overrun = sqrt(((target(1)-location(1))^2)+((target(2)-location(2))^2)); 
            location(:) = target(:);
        else
            overrun = 0;
        end
    elseif theta >= 180 && theta < 270
        if location(1) <= target(1) && location(2) <= target(2)
            overrun = sqrt(((target(1)-location(1))^2)+((target(2)-location(2))^2)); 
            location(:) = target(:);
        else
            overrun = 0;
        end
    elseif theta >= 270 && theta < 360
        if location(1) <= target(1) && location(2) >= target(2)
            overrun = sqrt(((target(1)-location(1))^2)+((target(2)-location(2))^2)); 
            location(:) = target(:);
        else
            overrun = 0;
        end
    end
end