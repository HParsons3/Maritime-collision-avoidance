clear all
close all

%Set Parameters
startA = [2,1]; %Start positions
startB = [1,1]; 
goalA = [1,2]; %Goal positions
goalB = [2,2];
speedA = 0.25; %Speeds
speedB = 0.25;
startloc = [startA;startB];
goalloc = [goalA;goalB];
speed = [speedA,speedB];
runSim2(startloc,goalloc,speed);

function runSim2(startloc, goalloc, speed)
    hold on 
    xlim([0,3]) %Set up graph- size changes may be necessary
    ylim([0,3])
    scatter(startloc(1,1),startloc(1,2),'dr') %Show starting positions
    scatter(startloc(2,1),startloc(2,2),'db')
    scatter(goalloc(1,1),goalloc(1,2),'or')
    scatter(goalloc(2,1),goalloc(2,2),'ob')
    path = zeros(2);
    pathstart = zeros(1,2);
    for n = 1:2 %For each entity
        [pathstart, theta(n)] = buildPath(startloc(n,:), goalloc(n,:), speed(n)); %Make a path from the start to the goal
        for i = 1:height(pathstart)
            path(n,:,i) = pathstart(i,:);
        end
    end
    [~,~,d] = size(path); %Plot the expected path
    for n = 1:2
        for x = 1:d
            line(x,:) = [path(n,1,x),path(n,2,x)];
            if line(x,:) == [0,0]
                line(x,:) = line(x-1,:);
            end
        end
        h1(n) = plot(line(:,1),line(:,2),'.');
    end
    flag1 = 0;
    for t = 1:(100-1) %Until the last path is finished, or it times out
        for n = 1:2 %For every entity
            overrun = zeros(n,1);
            if t > 1 %Start saving old locations after first tick
                [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, i, j, n, t);
%                 h2(n) = plot(points(:,1), points(:,2), '-');
            else
                location(n,:) = path(n,:,t);
                locationold = startloc;
            end
        end
%         for n = 1:2
%             for x = 1:d
%                 line(x,:) = [path(n,1,x),path(n,2,x)];
%                 if line(x,:) == [0,0]
%                     line(x,:) = line(x-1,:);
%                 end
%             end
%             h1(n) = plot(line(:,1),line(:,2),'.');
%         end
%        keyboard %To allow watching the movements step by step
%        delete(h1);
%         if t > 1
%             delete(h2);
%         end
    end
end