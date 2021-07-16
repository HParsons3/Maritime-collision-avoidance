%This function will create a line between two points A1 and A2, and a line
%between points B1 and B2, and find if they intersect.

function intersection = findIntersection(A1, A2, B1, B2)
    [a, b] = createLine(A1, A2); %Create lines between each of the points
    [c, d] = createLine(B1, B2);
    x = (d-b)/(a-c); %Calculate x point of intersection. Returns inf if no intersection.
%    y = (a*x)+b;
    if a == inf && ((A1(1) <= B1(1) && A1(1) >= B2(1)) || (A1(1) <= B2(1) && A1(1) >= B1(1))) %Check if the lines intersect
        intersection = 1;
    elseif c == inf && ((B1(1) <= A1(1) && B1(1) >= A2(1)) || (B1(1) <= A2(1) && B1(1) >= A1(1)))
        intersection = 1;
    elseif x ~= inf && ((x <= A1(1) && x >= A2(1)) || (x>= A1(1) && x <= A2(1))) && ((x <= B1(1) && x >= B2(1)) || (x>= B1(1) && x <= B2(1)))
        intersection = 1;
    else
        intersection = 0;
    end
end