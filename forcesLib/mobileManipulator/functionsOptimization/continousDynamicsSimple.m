function x_dot = continousDynamicsSimple(z, p)

x_base = z(1:3);
q = z(4:10);
q_dot = z(13:19);
u = z(11:12);
dt = p(1);
r = p(2);
L = p(3);


x_base_dot = [r/2 * (u(1) + u(2)) * cos(x_base(3));...
    r/2 * (u(1) + u(2)) * sin(x_base(3));...
    r/L * (u(1) - u(2))];
x_dot = [x_base_dot; q_dot];
end

