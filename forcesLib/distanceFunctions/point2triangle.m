function dist = point2triangle(a, b)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
A = b(1:3);
B = b(4:6);
C = b(7:9);
vec1 = B - A;
vec2 = C - A;

normal  = contCrossProduct(vec1, vec2);
area = sqrt(contDotProduct(normal, normal))/2;

% projection point P
alpha = contDotProduct(normal, a - A)/(contDotProduct(normal, normal) + eps);
P = a - normal * alpha;
d_o = point2point(P, a);

cpA = contCrossProduct(B - P, C - P);
cpB = contCrossProduct(C - P, A - P);
cpC = contCrossProduct(A - P, B - P);
t1 = sqrt(contDotProduct(cpA, cpA)) / ( (2 * area) + eps);
t2 = sqrt(contDotProduct(cpB, cpB)) / ( (2 * area) + eps);
t3 = sqrt(contDotProduct(cpC, cpC)) / ( (2 * area) + eps);
tges = t1 + t2 + t3;

tdiff = sqrt((tges - 1)^2);

d_c = point2line(P, [A; B]);
d_a = point2line(P, [B; C]);
d_b = point2line(P, [A; C]);

k = 100;
s_inside = sigmoid(t1, 0, k) * sigmoid(t1, 1, -k) * sigmoid(t2, 0, k) * sigmoid(t2, 1, -k) * sigmoid(t3, 0, k) * sigmoid(t3, 1, -k) * sigmoid(tdiff, 0.1, -k);

d_sides = contMin(d_a, d_b, d_c) * (1 - s_inside);
dist = sqrt(d_o^2 + d_sides^2);

end

