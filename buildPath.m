function [path, theta] = buildPath(startpoint, goalpoint, speed)
    o = goalpoint(2)-startpoint(2); 
    a = goalpoint(1)-startpoint(1);
%   h = sqrt(a^2+o^2); %Distance to target
    theta = atand(o/a); 
    if a <= 0 && o <= 0 %Angle to target
        theta = 270-theta;
    elseif a <= 0 && o >= 0
        theta = 270-theta;
    elseif a >= 0 && o <= 0
        theta = 90-theta;
    elseif a >= 0 && o >= 0
        theta = 90-theta;
    end
    path = zeros(1,2);
    i = 1;
    readyflag = 0;
    while readyflag == 0 && i < 100
        readyflag = 1;
        path(i,:) = [startpoint(1)+sind(theta)*speed*i, startpoint(2)+cosd(theta)*speed*i]; %Move along the line step by step
        %If the goal has been passed, remain on the goal point
        if theta >= 0 && theta < 90 
            if path(i,1) >= goalpoint(1) && path(i,2) >= goalpoint(2)
                path(i,:) = goalpoint(:);
            else
                readyflag = 0;
            end
        elseif theta >= 90 && theta < 180
            if path(i,1) >= goalpoint(1) && path(i,2) <= goalpoint(2)
                path(i,:) = goalpoint(:);
            else
                readyflag = 0;
            end
        elseif theta >= 180 && theta < 270
            if path(i,1) <= goalpoint(1) && path(i,2) <= goalpoint(2)
                path(i,:) = goalpoint(:);
            else
                readyflag = 0;
            end
        elseif theta >= 270 && theta < 360
            if path(i,1) <= goalpoint(1) && path(i,2) >= goalpoint(2)
                path(i,:) = goalpoint(:);
            else
                readyflag = 0;
            end
        end
        i = i+1;
    end
end