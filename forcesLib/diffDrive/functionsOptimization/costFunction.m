function J = costFunction(z, p)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
q = z(1:3);
u = z(4:5);
g = p(4:6);

W = diag([100, 100, 0]);
P = diag([1 1]);

J = (q - g)' * W * (q - g) + u' * P * u;

end

