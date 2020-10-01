function dist = point2line(a, b)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    bs = b(1:3);
    be = b(4:6);
    
    vec = be - bs;
    alpha = contDotProduct(a - bs, vec)/contDotProduct(vec, vec);
    t = bs + alpha * vec;
    d_o = point2point(t, a);
    d_s = point2point(t, bs);
    d_e = point2point(t, be);
    
    gain = 100;
    dist = sqrt(d_o^2 + (sigmoid(alpha, 0, -gain) * d_s)^2 + (sigmoid(alpha, 1, gain) * d_e)^2); 
end

