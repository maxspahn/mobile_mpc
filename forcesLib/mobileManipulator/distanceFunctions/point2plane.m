function dist = point2plane(a,b)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

bs = b(1:3);
be1 = b(4:6);
be2 = b(7:9);
vec1 = be1 - bs;
vec2 = be2 - bs;
bee = bs + vec1 + vec2;

tri1dist = point2triangle(a, b);
tri2dist = point2triangle(a, [be1; be2; bee]);

dist = contMin(tri1dist, tri2dist);


end

