function dist = point2infplane(p, Ab)
% Ab = [A, b];
%Give a constraint of plane form Ap < b
% inwards directed normal, towards free space
n = Ab(1:3);
b = Ab(4);


alpha = (contDotProduct(p, n) - b)/(contDotProduct(n, n) + eps);
P = p - alpha * n;

% include sign
k = 100;
dist = point2point(P, p);
dist = dist * sigmoid(alpha, 0, k);
%- dist * sigmoid(alpha, 0, -k);

end

