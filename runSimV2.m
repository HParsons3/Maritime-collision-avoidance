clear all
close all

%Set Parameters
goalA = [3,0]; %Start positions
startB = [2.9,0]; 
startA = [0.1,0]; %Goal positions
goalB = [0,0];
speedA = 0.25; %Speeds
speedB = 0.25;
startloc = [startA;startB];
goalloc = [goalA;goalB];
speed = [speedA,speedB];
runSim2(startloc,goalloc,speed);

function runSim2(startloc, goalloc, speed)
    hold on 
    xlim([-1,5]) %Set up graph- size changes may be necessary
    ylim([-1,5])
    scatter(startloc(1,1),startloc(1,2),'dr') %Show starting positions
    scatter(startloc(2,1),startloc(2,2),'db')
    scatter(goalloc(1,1),goalloc(1,2),'or')
    scatter(goalloc(2,1),goalloc(2,2),'ob')
    for shipID = 1:2 %For each entity
        [pathstart, theta(shipID)] = buildPath(startloc(shipID,:), goalloc(shipID,:), speed(shipID)); %Make a path from the start to the goal
        for pathsize = 1:height(pathstart)
            path(shipID,:,pathsize) = pathstart(pathsize,:);
        end
        h1(shipID) = plot(pathstart(:,1),pathstart(:,2),'.');
    end
    for t = 1:(100-1) %Until the last path is finished, or it times out
        for shipID = 1:2 %For every entity
            if t > 1 %Start saving old locations after first tick
                [~, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, pathsize, j, shipID, t);
            else
                location(shipID,:) = path(shipID,:,t);
                locationold = startloc;
            end
        end
        [~,~,d] = size(path); %Plot the expected path
        for shipID = 1:2
            for x = 1:d
                line(x,:) = [path(shipID,1,x),path(shipID,2,x)];
                if line(x,:) == [0,0]
                    line(x,:) = line(x-1,:);
                end
            end
            h1(shipID) = plot(line(:,1),line(:,2),'.');
        end
         keyboard
    end
end