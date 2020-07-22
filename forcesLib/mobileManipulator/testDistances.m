addpath('./distanceFunctions');

EPSILON = 5e-4;
line = [0, 0, 0, 3, 3, 0]';
line2 = [-5, -2, -4, 7, 1, 11]';
plane = [1, 1, 0, 5, 1, 0, 1, 4, 0]';
plane2 = [1, 1, 0, 5, 1, 0, 3, 3, 0]';
plane3 = [1, 3, 0, 4, 8, 0, 6, 3, 0]';
plane4 = [-2, -5.5, 0, 0, -5.5, 0, -2, -5.5, 0.7]';
plane5 = [-2, -5.5, 0, -2, -7.5, 0, -2, -5.5, 0.7]';
planeInf = [0, 0, 1, 1]';
planeInf2 = [-0.5, 1, 0, -1]';
triangle = [1, 1, 0, 5, 1, 0, 1, 4, 0]';

%% pointOnLine
p1 = [3, 0, 0]';
d1 = point2line(p1, line);
assert(abs(d1 - 3/sqrt(2)) < EPSILON);

%% pointBesideLine
p2 = [-3, 0, 0]';
d2 = point2line(p2, line);
assert(abs(d2 - 3) < EPSILON);

%% pointLineEndCase
p3 = [6, 2, 0]';
d3 = point2line(p3, line);
d = sqrt(1 + 9);
assert(abs(d3 - d) < EPSILON);

%% pointBeside3DLine
p4 = [1, -0.5, 2]';
d4 = point2line(p4, line2);
assert(abs(d4 - 0.954313) < EPSILON);

%% pointInTriangle
p14 = [2, 2, 0]';
d14 = point2triangle(p14, triangle);
assert(abs(d14 - 0) < EPSILON);

%% pointLeftOfTriangle
p15 = [0, 2, 0]';
d15 = point2triangle(p15, triangle);
assert(abs(d15 - 1) < EPSILON);

%% pointBelowOfTriangle
p16 = [2, 0, 0]';
d16 = point2triangle(p16, triangle);
assert(abs(d16 - 1) < EPSILON);

%% pointAboveTriangle
p17 = [4.5, 4.5, 0]';
d17 = point2triangle(p17, triangle);
assert(abs(d17 - sqrt(4 + 1.5^2)) < EPSILON);

%% pointCornerPlaneSimple
p5 = [0, 0, 0]';
d5 = point2plane(p5, plane);
assert(abs(d5 - sqrt(2)) < EPSILON);

%% pointBelowPlane
p6 = [4, 0, 0]';
d6 = point2plane(p6, plane);
assert(abs(d6 - 1.0) < EPSILON);

%% pointCornerPlaneStraight
p7 = [0, 0, 0]';
d7 = point2plane(p7, plane2);
assert(abs(d7 - sqrt(2)) < EPSILON);

%% pointCornerPlane
p8 = [-0.5, 0.5, 0]';
d8 = point2plane(p8, plane2);
assert(abs(d8 - sqrt(1.5^2 + 0.5^2)) < EPSILON);

%% pointNextToPlane
p9 = [0, 3, 0]';
d9 = point2plane(p9, plane2);
assert(abs(d9 - sqrt(1.5^2 + 1.5^2)) < EPSILON);

%% pointAbovePlane
p10 = [3, 2, 3]';
d10 = point2plane(p10, plane2);
assert(abs(d10 - 3) < EPSILON);

%% pointBehindPlane
p11 = [8.5, 3.5, -3]';
d11 = point2plane(p11, plane2);
assert(abs(d11 - sqrt(3^2 + 1.5^2 + 0.5^2)) < EPSILON);

%% pointInPlane
p12 = [4, 2, 0]';
d12 = point2plane(p12, plane2);
assert(abs(d12 - 0) < EPSILON);

%% pointCornerPlaneBigAngle
p13 = [6, 0.5, 0]';
d13 = point2plane(p13, plane2);
assert(abs(d13 - 3/sqrt(8)) < EPSILON);

%% pointInPlane3
p18 = [5, 4.5, 0]';
d18 = point2plane(p18, plane3);
assert(abs(d18 - 0) < EPSILON);

%% pointAbovePlane3
p19 = [5, 4.5, 3]';
d19 = point2plane(p19, plane3);
assert(abs(d19 - 3) < EPSILON);

%% pointTableHorizontalOrigin
p20 = [0, 0, 0]';
d20 = point2plane(p20, plane4);
assert(abs(d20 - 5.5) < EPSILON);

%% pointTableVerticalOrigin
p21 = [2.5, 3.7, 1.0]';
d21 = point2plane(p21, plane5);
assert(abs(d21 - sqrt((2 + 2.5)^2 + (3.7 + 5.5)^2 + 0.3^2)) < EPSILON);

%% point2infPlane
p22 = [5, 4.5, 3]';
d22 = point2infplane(p22, planeInf);
assert(abs(d22 - 2) < EPSILON);

%% point2infPlaneNegative
p23 = [5, 4.5, -3]';
d23 = point2infplane(p23, planeInf);
assert(abs(d23 + 0) < EPSILON);

%% point2infPlane2Negative
p24 = [1, 2, 0]';
d24 = point2infplane(p24, planeInf2);
assert(abs(d24 - sqrt(5)) < EPSILON);

%% point2infPlane2
p25 = [3, -2, 0]';
d25 = point2infplane(p25, planeInf2);
assert(abs(d25 - 0) < EPSILON);






