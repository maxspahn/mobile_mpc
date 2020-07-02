function x_next = transitionFunctionAcc(z, p)

% x_base = z(1:3);
% u = z(4:5);
% q = z(6:12);
% q_dot = z(13:19);
% slack = z(20);
% u_dot = z(21:22);
% q_dotdot = z(23:29);
% dt = p(1);
% r = p(2);
% L = p(3);
% x_base_next = [x_base(1) + r/2 * (u(1) + u(2)) * cos(x_base(3)) * dt;
%     x_base(2) + r/2 * (u(1) + u(2)) * sin(x_base(3)) * dt;
%     x_base(3) + r/(¤ * L) * (- u(1) + u(2)) * dt];
% u_next = u + dt * u_dot;
% q_next = q + dt * q_dot;

%x_next = [x_base_next; u_next; q_next];
dt = p(1);
x = z(1:19);
u = z(21:29);
x_next = RK4(x, u, @continuousDynamics, dt);
end

function x_dot = continuousDynamics(x, u)
    x_base = x(1:3);
    u_base = x(4:5);
    q = x(6:12);
    q_dot = x(13:19);
    u_dot = u(1:2);
    q_dotdot = u(3:9);
    r = 0.08;
    L = 0.544;
    x_base_dot = [r/2 * (u_base(1) + u_base(2)) * cos(x_base(3));...
        r/2 * (u_base(1) + u_base(2)) * sin(x_base(3));...
        r/(2 * L) * (- u_base(1) + u_base(2))];
    x_dot = [x_base_dot; u_dot; q_dot; q_dotdot];
end


