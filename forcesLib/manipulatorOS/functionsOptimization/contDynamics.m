function x_dot = contDynamics(z, q_dot)
    load('functionHandle.mat')
    x_dot = ht1(z(1), z(2), z(3), z(4), z(5), z(6), z(7), z(8), z(9), z(10)) * q_dot;
end
