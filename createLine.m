%This function creates a line between pointA and pointB

function [m, c] = createLine(pointA, pointB)
    if pointA(1)-pointB(1) == 0 %If vertical line, gradient is infinite
        m = inf; 
    else %Otherwise, calculate gradient
        m = (pointB(2)-pointA(2))/(pointB(1)-pointA(1)); 
    end
    if m == inf %If vertical line, c does not exist
        c = inf;
    else %Otherwise, calculate c
        c = pointA(2)-(m*pointA(1));
    end
end