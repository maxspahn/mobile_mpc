function m = contMin(a, b, c)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    narginchk(1,3) 
    if nargin >= 2
        s = sigmoid((a - b), 0, 1000);
        m = s * b + (1 - s) * a;
    end
    if nargin >= 3
        s2 = sigmoid((m - c), 0, 1000);
        m = s2 * c + (1 - s2) * m;
    end
end
