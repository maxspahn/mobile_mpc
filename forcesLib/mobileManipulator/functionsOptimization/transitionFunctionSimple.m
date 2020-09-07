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
    offsetBase = 0.247;
    r = 0.08;
    %L = 0.544;
    L = 0.494;
    % Compute velocities around origin (point in between wheels
    angVel = r/(2*L) * (-u_base(1) + u_base(2));
    xVel = r/2 * (u_base(1) + u_base(2)) * cos(x_base(3));
    yVel =  r/2 * (u_base(1) + u_base(2)) * sin(x_base(3));
    % Transform to center of base
    x_base_dot = [xVel - angVel * offsetBase * sin(x_base(3));...
        yVel + angVel * offsetBase * cos(x_base(3));...
        angVel];
    x_base_dot = [xVel; yVel; angVel];
    x_dot = [x_base_dot; q_dot];
end

