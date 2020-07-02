close all;
clear;
clc;

%% figure
% fig1 = figure(1);
% ax1 = axes('Parent', fig1, 'xlim', [-5, 5], 'ylim', [-5, 5], 'zlim', [0, 5]);
% view(ax1, 3);
% grid on;
% hold(ax1, 'on');

fig2 = figure(2);
ax2 = axes('Parent', fig2);
hold(ax2, 'on');

%%
% objectMatrix = makehgtform('translate', [1, 1, 1]);
% ht1 = hgtransform('Parent', ax1, 'Matrix', objectMatrix);
% [x, y, z] = sphere(10);
% 
% surf('Parent', ht1, x, y, z);



%%
H = 15;

dt = 0.05;
base_pos = [0; 0; pi/4];
arm_pos = [0; 0; 0; -1; 0.5; 1.5; 0];
u_start = zeros(9, 1);
start = [base_pos; arm_pos; u_start];
goal_base_pos = [5; 5; pi/4];
goal_arm_pos = [1; 1; -0.5; -1.5; 0.5; 1; 0.2];

goal = [goal_base_pos; goal_arm_pos];

problem.xinit = start;
problem.x0 = repmat(start, H, 1);

obstacles = [2, 2.5, 0, 1];

% rectangle('Parent', ax2, 'Position', [obstacles(1) - obstacles(4) obstacles(2) - obstacles(4) 2 * obstacles(4) 2 * obstacles(4)], 'Curvature', 1);
% ht1 = hgtransform('Parent', ax2);
% posMatrix = makehgtform('translate', [start(1:2)', 0]) * makehgtform('zrotate', start(3));
% set(ht1, 'Matrix', posMatrix);
% rectangle('Parent', ht1, 'Position', [-0.5, -0.5, 1, 1]);

%% Converting obstacles and fill up

r = 0.1;
L = 0.5;
problem.all_parameters = repmat([dt, r, L, goal', obstacles], 1, H)';

curState = start;
error = 10000;
t = 0;
while 1
    [output, exitflag, info] = mm_MPC(problem);
    curState = output.x02(1:10);
    problem.xinit = output.x02;
    problem.x0 = repmat(output.x02, H, 1);
    
    disp(curState);
    plot(ax2, curState(1), curState(2), 'ro');
    pause(0.1);
    
    oldError = error;
    error = norm(curState - goal);
    if error < 0.1 || oldError <= 0.1 * error
        break;
    end
    t = t + dt;
  
end


