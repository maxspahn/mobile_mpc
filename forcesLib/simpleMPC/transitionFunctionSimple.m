function x_next = transitionFunctionSimple(z, p, pMap)

dt = p(pMap.dt);

x = z(1:10);
u = z(12:20);
x_next = RK4(x, u, @continuousDynamics, dt);
end

function x_dot = continuousDynamics(x, u)
    x_base = x(1:3);
    q = x(4:10);
    u_base = u(1:2);
    q_dot = u(3:9);
    r = 0.08;
    L = 0.494;
    linearVel = 0.5 * (u_base(1) + u_base(2)) * r;
    angVel = r/L * (-u_base(1) + u_base(2));
    xVel = linearVel * cos(x_base(3));
    yVel =  linearVel * sin(x_base(3));
    x_base_dot = [xVel; yVel; angVel];
    x_dot = [x_base_dot; q_dot];
end

