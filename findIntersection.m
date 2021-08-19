%This function will create a line between two points A1 and A2, and a line
%between points B1 and B2, and find if they intersect.

function intersection = findIntersection(A1, A2, B1, B2)
%    keyboard
    [a, b] = createLine(A1, A2); %Create lines between each of the points
    [c, d] = createLine(B1, B2);
    x = (d-b)/(a-c); %Calculate x point of intersection. Returns inf if no intersection.
    y = (a*x)+b;
    if (c == inf) &&(a == inf) && (((A1(2) <= B1(2) && A1(2) >= B2(2)) || (A1(2) <= B2(2) && A1(2) >= B1(2)) || (A2(2) <= B1(2) && A2(2) >= B2(2)) || (A2(2) <= B2(2) && A2(2) >= B1(2))) || ((B1(2) <= A1(2) && B1(2) >= A2(2)) || (B1(2) <= A2(2) && B1(2) >= A1(2)) || (B2(2) <= A1(2) && B2(2) >= A2(2)) || (B2(2) <= A2(2) && B2(2) >= A1(2)))) %Check if the lines intersect
        intersection = 1;
    elseif (c == 0) && (a == 0) && (((A1(1) <= B1(1) && A1(1) >= B2(1)) || (A1(1) <= B2(1) && A1(1) >= B1(1)) || (A2(1) <= B1(1) && A2(1) >= B2(1)) || (A2(1) <= B2(1) && A2(1) >= B1(1))) || ((B1(1) <= A1(1) && B1(1) >= A2(1)) || (B1(1) <= A2(1) && B1(1) >= A1(1)) || (B2(1) <= A1(1) && B2(1) >= A2(1)) || (B2(1) <= A2(1) && B2(1) >= A1(1))))
        intersection = 1;
    elseif x ~= inf && ((x <= A1(1) && x >= A2(1)) || (x>= A1(1) && x <= A2(1))) && ((x <= B1(1) && x >= B2(1)) || (x>= B1(1) && x <= B2(1)))
        intersection = 1;
    else
        intersection = 0;
    end
    %If the lines stem from the same point, they can incorrectly show as
    %intersecting. This should prevent that.
    if ((x <= A1(1)+0.0001 && x >= A1(1)-0.0001) || (x <= A2(1)+0.0001 && x >= A2(1)-0.0001) || (x <= B1(1)+0.0001 && x >= B1(1)-0.0001) || (x <= B2(1)+0.0001 && x >= B2(1)-0.0001)) && ((y <= A1(2)+0.0001 && y >= A1(2)-0.0001) || (y <= A2(2)+0.0001 && y >= A2(2)-0.0001) || (y <= B1(2)+0.0001 && y >= B1(2)-0.0001) || (y <= B2(2)+0.0001 && y >= B2(2)-0.0001))
        intersection = 0;
    end
end