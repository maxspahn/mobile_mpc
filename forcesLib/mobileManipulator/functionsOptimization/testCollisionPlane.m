%%
close all;
clear all
clc;

%%

fig1 = figure(1);
ax1 = axes('Parent', fig1, 'xlim', [-10, 10], 'ylim' , [-10, 10]);
%axis equal
hold(ax1, 'on')
fig2 = figure(2);
ax2 = axes('Parent', fig2);


plane = [1, 1, 0, 3, 1, 0, 1, 1, 3]';
plane = [-5, -5, 1, -5, 5, 1, 5, -5, 1]';
plane = [4, 3, 0.2, 4, 8, 0.2, 10, 3, 0.2]';


[X, Y] = meshgrid(-10:0.1:10,-10:0.1:10);
Z = nan(size(X));
for i=1:size(X, 1)
    for j=1:size(X,2)
        sphere = [X(i,j), Y(i,j), 0.2, 2.5]';
        Z(i, j)= point2plane(sphere(1:3), plane);
    end
end


Z_valid = Z .* (Z > 0);
surfc('Parent', ax2, X, Y, Z);
%%
sphere = [5, -1 0, 0.5]';

xp = -10:0.25:40;
yp = planeEquation(plane, xp);

plot(ax1, xp, yp);



%rectangle('Parent', ax1, 'Position', [sphere(1) - sphere(4) sphere(2) - sphere(4) 2 * sphere(4) 2 * sphere(4)], 'Curvature', 1);

a = planeCollisionAvoidance(plane, sphere);
disp("if greater than zero then no collision");
disp(a);

function y = planeEquation(plane, x)
    y = nan(1, size(x, 2));
    for i=1:size(x,2)
        y(i) = (dot(plane(1:3), plane(4:6)) - plane(1) * x(i))/plane(2);
        a = norm([x(i); y(i)] - plane(4:5));
        if a > plane(7)
            y(i) = NaN;
        end
    end
end
    

