function x_next = transitionFunctionTorques(z, p)

x_base = z(1:3);
q = z(4:10);
q_dot = z(11:17);
u = z(18:19);
tau = z(20:26);
dt = p(1);
r = p(2);
L = p(3);

M = get_MassMatrix(q);
c = get_CoriolisVector(q,q_dot);
g = get_GravityVector(q);

x_base_next = [x_base(1) + r/2 * (u(1) + u(2)) * cos(x_base(3)) * dt;
    x_base(2) + r/2 * (u(1) + u(2)) * sin(x_base(3)) * dt;
    x_base(3) + r/L * (u(1) - u(2)) * dt];
q_next = q + dt * q_dot;
q_dot_next = q_dot + dt * (pinv(M) * (tau - c + g));

x_next = [x_base_next; q_next; q_dot_next];
end

