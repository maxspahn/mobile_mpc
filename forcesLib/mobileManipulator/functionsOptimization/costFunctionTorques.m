function J = costFunctionTorques(z, p)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
q = z(4:10);
q_dot = z(11:17);
x_des = p(4:6);
q_des = p(7:13);
q_dot_des = p(14:20);
u = z(18:19);
tau = z(20:26);

Wq = diag(ones(size(q, 1), 1) * 100);
Wqdot = diag(ones(size(q_dot, 1), 1) * 100);
Wx = diag(ones(size(x_des, 1), 1) * 100);
% Turning of rotation of base
Wx(3, 3) = 0;
Pu = diag(ones(size(u, 1), 1));
Ptau = diag(ones(size(tau, 1), 1));

J = (x - x_des)' * Wx * (x - x_des) + ...
    (q - q_des)' * Wq * (q - q_des) + ...
    (q_dot - q_dot_des)' * Wqdot * (q_dot - q_dot_des) + ...
    u' * Pu * u + ...
    tau' * Ptau * tau;
    

end

