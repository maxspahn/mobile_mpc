function x_next = transitionFunction(z, p)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
u = z(4:5);
dt = p(1);
r = p(2);
L = p(3);
x_next = [x(1) + r/2 * (u(1) + u(2)) * cos(x(3)) * dt;
    x(2) + r/2 * (u(1) + u(2)) * sin(x(3)) * dt;
    x(3) + r/L * (u(1) - u(2)) * dt];
end

