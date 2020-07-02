function x_next = transitionFunctionSimple(z, p)

x_base = z(1:3);
q = z(4:10);
slack = z(11);
u = z(12:13);
q_dot = z(14:20);
dt = p(1);
r = p(2);
L = p(3);
x_des = p(4:6);
q_des = p(7:13);
x_base_next = [x_base(1) + r/2 * (u(1) + u(2)) * cos(x_base(3)) * dt;
    x_base(2) + r/2 * (u(1) + u(2)) * sin(x_base(3)) * dt;
    x_base(3) + r/(2 * L) * (- u(1) + u(2)) * dt];
q_next = q + dt * q_dot;

x = z(1:10);
u = z(12:20);
%x_next = [x_base_next; q_next];
x_next = RK4(x, u, @continuousDynamics, dt);
end

function x_dot = continuousDynamics(x, u)
    x_base = x(1:3);
    q = x(4:10);
    u_base = u(1:2);
    q_dot = u(3:9);
    r = 0.08;
    L = 0.544;
    x_base_dot = [r/2 * (u_base(1) + u_base(2)) * cos(x_base(3));...
        r/2 * (u_base(1) + u_base(2)) * sin(x_base(3));...
        r/(2 * L) * (- u_base(1) + u_base(2))];
    x_dot = [x_base_dot; q_dot];
end

