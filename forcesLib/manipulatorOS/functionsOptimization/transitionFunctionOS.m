function x_next = transitionFunctionOS(z, p)

x_base = z(1:3);
q = z(4:10);
q_dot = z(13:19);
u = z(11:12);
dt = p(1);
r = p(2);
L = p(3);
x_base_next = [x_base(1) + r/2 * (u(1) + u(2)) * cos(x_base(3)) * dt;
    x_base(2) + r/2 * (u(1) + u(2)) * sin(x_base(3)) * dt;
    x_base(3) + r/L * (u(1) - u(2)) * dt];
q_next = q + dt * q_dot;

x_R_next = RK4(z(1:10), q_dot, @continuousDynamics, dt);
x_next = x_R_next;
end



