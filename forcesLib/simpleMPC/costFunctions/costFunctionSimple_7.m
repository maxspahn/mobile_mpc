function J = costFunctionSimple_7(z, p, pMap)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
q = z(4:10);
slack = z(11);
u = z(12:13);
q_dot = z(14:20);

% Ugly workaround as passing i as parameter is not supported?!
i = 7;

path_index_x = pMap.refPath(1) + 3 * (i-1);
path_index_o = path_index_x + 2;
x_des = p(path_index_x:path_index_o);

q_des = p(pMap.desArmConfig(1): pMap.desArmConfig(2));

weights = p(pMap.weights(1):pMap.weights(2));
wq = weights(1);
wc = weights(2);
wl = weights(3);
wo = weights(4);
wslack = weights(5);
wpu = weights(6);
wpqdot = weights(7);


Wq = diag(ones(size(q, 1), 1) * wq);
Wc = wc;
Wl = wl;
% Turning of rotation of base
Wo = wo;


Wslack = wslack;
Wu = diag(ones(size(u, 1), 1) * wpu);
Pqdot = diag(ones(size(q_dot, 1), 1) * wpqdot);
% avoid Turning cost

%contourGoal = computeGoal([x; q], [x_des; q_des], i, dt);
% Compute lag and contour error
dx = x_des(1) - x(1);
dy = x_des(2) - x(2);
d = [dx; dy];

% Rotate into the path reference rotation
% Check reference frames
% Currently rotation around z axis assumed
R = [cos(x_des(3)), sin(x_des(3));...
     -sin(x_des(3)), cos(x_des(3))];
d_rot = R * d;
err_contour = d_rot(2);
err_lag = d_rot(1);
%err_contour = dx;
%err_lag = dy;



J = err_contour' * Wc * err_contour + ...
    err_lag' * Wl * err_lag + ...
    (x(3) - x_des(3))' * Wo * (x(3) - x_des(3)) + ...
    (q - q_des)' * Wq * (q - q_des) + ...
    u' * Wu * u + ...
    q_dot' * Pqdot * q_dot + ...
    slack * Wslack * slack;
end


