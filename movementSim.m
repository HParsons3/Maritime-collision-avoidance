function [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, i, j, n, t)
%        keyboard
        locationold(n,:) = location(n,:); %Set previous location
        location(n,:) = path(n,:,t); %Move along the path
        points = [0, 0];
        o = location(n,2)-locationold(n,2); 
        a = location(n,1)-locationold(n,1);
    %   h = sqrt(a^2+o^2); %Distance to target
        theta(n) = atand(o/a); 
        if a <= 0 && o <= 0 %Angle to target
            theta(n) = 270-theta(n);
        elseif a <= 0 && o >= 0
            theta(n) = 270-theta(n);
        elseif a >= 0 && o <= 0
            theta(n) = 90-theta(n);
        elseif a >= 0 && o >= 0
            theta(n) = 90-theta(n);
        end
        if location(n,:) == [0,0] %If the path is complete, remain in place
            location(n,:) = locationold(n,:);
        end
        if n > 1
            if locationold(n,:) == location(n,:) %If it is stationary, don't do anything
                points(n,:,:) = location(n,:);
            else
                distance = sqrt(((locationold(n,1)-location(n,1))^2)+((locationold(n,2)-location(n,2))^2)); 
                if distance < speed(n)
                    overrun = speed(n)-distance;
                else
                    overrun = 0;
                end
                count = 1;
                if overrun > 0.001
                    if location(n,:) == goalloc(n,:)
                    else
                        j = 1;
                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                        points = [locationold(n,:); location(n,:)];
                        locationold(n,:) = location(n,:);
                        locationdir = path(n,:,t+1);
                        o = locationdir(2)-locationold(n,2); 
                        a = locationdir(1)-locationold(n,1);
                    %   h = sqrt(a^2+o^2); %Distance to target
                        theta(n) = atand(o/a); 
                        if a <= 0 && o <= 0 %Angle to target
                            theta(n) = 270-theta(n);
                        elseif a <= 0 && o >= 0
                            theta(n) = 270-theta(n);
                        elseif a >= 0 && o <= 0
                            theta(n) = 90-theta(n);
                        elseif a >= 0 && o >= 0
                            theta(n) = 90-theta(n);
                        end
                        location(n,:) = [locationold(n,1)+(sind(theta(n))*overrun), locationold(n,2)+cosd(theta(n))*overrun];
                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                        points = [points; location(n,:)];
                        distance = sqrt(((locationold(n,1)-location(n,1))^2)+((locationold(n,2)-location(n,2))^2)); 
                        if distance < speed(n)
                            overrun = speed(n)-distance;
                        else
                            overrun = 0;
                        end
                    end
                    count = count+1;
                else
                count = 1;
                    for j = 1:n-1 %For every ship that has already been computed
                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                    end
                    points = [locationold(n,:); location(n,:)];
                end
            end
        else 
            distance = sqrt(((locationold(n,1)-location(n,1))^2)+((locationold(n,2)-location(n,2))^2)); 
                if distance < speed(n)
                    overrun = speed(n)-distance;
                else
                    overrun = 0;
                end
                if overrun > 0.001
                    if location(n,:) == goalloc(n,:)
                    else
%                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                        points = [locationold(n,:); location(n,:)];
                        locationold(n,:) = location(n,:);
                        locationdir = path(n,:,t+1);
                        o = locationdir(2)-locationold(n,2); 
                        a = locationdir(1)-locationold(n,1);
                    %   h = sqrt(a^2+o^2); %Distance to target
                        theta(n) = atand(o/a); 
                        if a <= 0 && o <= 0 %Angle to target
                            theta(n) = 270-theta(n);
                        elseif a <= 0 && o >= 0
                            theta(n) = 270-theta(n);
                        elseif a >= 0 && o <= 0
                            theta(n) = 90-theta(n);
                        elseif a >= 0 && o >= 0
                            theta(n) = 90-theta(n);
                        end
                        location(n,:) = [locationold(n,1)+(sind(theta(n))*overrun), locationold(n,2)+cosd(theta(n))*overrun];
%                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                        points = [points; location(n,:)];
                    end
                else
                    for j = 1:n-1 %For every ship that has already been computed
                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                    end
                    points = [locationold(n,:); location(n,:)];
                end
        end
        [~,~,d] = size(path); %Plot the expected path
        for k = 1:2
            for x = 1:d
                line(x,:) = [path(k,1,x),path(k,2,x)];
                if line(x,:) == [0,0]
                    line(x,:) = line(x-1,:);
                end
            end
            h1(k) = plot(line(:,1),line(:,2),'.');
        end
        if points ~= [0,0]
            h2(n) = plot(points(:,1), points(:,2), '-');
        end
        keyboard
end