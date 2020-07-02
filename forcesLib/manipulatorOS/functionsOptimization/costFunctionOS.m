function J = costFunctionOS(z, p)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
q = z(4:10);
x_ee = z(11:13);
u = z(14:15);
q_dot = z(15:22);
x_ee_des = p(3:5);
u = z(11:12);
Wee = diag(ones(size(x_ee, 1), 1) * 1000);
% Turning of rotation of base
Pu = diag(ones(size(u, 1), 1) * 0);
Pqdot = diag(ones(size(q_dot, 1), 1) * 0);

J = (x_ee - x_ee_des)' * Wee * (x_ee - x_ee_des) + ...
    u' * Pu * u + ...
    q_dot' * Pqdot * q_dot;
    

end

