close all;
clear;
clc;

q = sym('q', [7, 1]);
x = sym('x', [3,1]);

Ts = forwardKinematicsExp(q, x);

T_fin = ones(4,4);

for i = 1:4:size(Ts, 2)
    disp(Ts(:,i:i+3))
    T_fin = Ts(:,i:i+3) * T_fin;
    disp(i)
end

x = T_fin(1:3,4);

%j = jacobian(x)

ht1 = matlabFunction(jacobian(x));
clear x T_fin q x i Ts

