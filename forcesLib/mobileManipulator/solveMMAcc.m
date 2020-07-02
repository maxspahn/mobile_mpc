%close all;
clear;
clc;

%% figure
% fig1 = figure(1);
% ax1 = axes('Parent', fig1, 'xlim', [-5, 5], 'ylim', [-5, 5], 'zlim', [0, 5]);
% view(ax1, 3);
% grid on;
% hold(ax1, 'on');

fig2 = figure(2);
ax2 = axes('Parent', fig2, 'xlim', [-5, 15], 'ylim', [-5, 15]);
hold(ax2, 'on');

%%
% objectMatrix = makehgtform('translate', [1, 1, 1]);
% ht1 = hgtransform('Parent', ax1, 'Matrix', objectMatrix);
% [x, y, z] = sphere(10);
% 
% surf('Parent', ht1, x, y, z);



%%
H = 20;

dt = 0.3;
base_pos = [0; 0; 1];
u_base_start = [0; 0];
arm_pos = [0; 0; 0; -1; 0.5; 1.5; 0];
arm_vel = zeros(7, 1);
u_start = zeros(9, 1);
start = [base_pos; u_base_start; arm_pos; arm_vel];
goal_base_pos = [9; 9; pi/4];
goal_arm_pos = [1; 1; -0.5; -1.5; 0.5; 1; 0.2];

goal = [goal_base_pos; goal_arm_pos];

problem.xinit = [start; 0; u_start];
problem.x0 = repmat([start; u_start; 0], H, 1);

obstacles = ones(5 * 4, 1) * -100;
obstacles(1:4) = [4; 5.5; 0; 4];
obstacles(5:8) = [7; 4.5; 0; 3];
obstacles(9:12) = [8; 7; 0; 0.1];
% wq, wx, wo, wslack, wpu, wpqdot, wpudot, wpqdotdot 
weights = [1, 100, 0, 100000000, 0, 0, 2, 0];
for i=1:3
    rectangle('Parent', ax2, 'Position', [obstacles(4 * (i-1) + 1) - obstacles(4 * (i-1) + 4) obstacles(4 * (i-1) + 2) - obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4)], 'Curvature', 1);
end
plot(goal_base_pos(1), goal_base_pos(2), 'rx');
% ht1 = hgtransform('Parent', ax2);
% posMatrix = makehgtform('translate', [start(1:2)', 0]) * makehgtform('zrotate', start(3));
% set(ht1, 'Matrix', posMatrix);
% rectangle('Parent', ht1, 'Position', [-0.5, -0.5, 1, 1]);

%% Converting obstacles and fill up

r = 0.1;
L = 0.9;
params = repmat([dt, r, L, goal', weights, obstacles'], 1, H)';
problem.all_parameters = params;

curState = start;
newState = start;
error = 10000;
t = 0;
while 1
    [output, exitflag, info] = mm_MPC(problem);
    curState = output.x02(1:3);
    curU = output.x01(21:29);
    disp(output.x02(20));
    problem.xinit = output.x02;
    problem.x0 = [output.x02;...
        output.x03;...
        output.x04;...
        output.x05;...
        output.x06;...
        output.x07;...
        output.x08;...
        output.x09;...
        output.x10;...
        output.x11;...
        output.x12;...
        output.x13;...
        output.x14;...
        output.x15;...
        output.x16;...
        output.x17;...
        output.x18;...
        output.x19;...
        output.x20;...
        output.x20;...
        ];
    %problem.x0 = repmat(output.x01, H, 1);
    
    disp(exitflag);
    newState = dynamics_mm(curU, newState, dt, r, L);
    plot(ax2, curState(1), curState(2), 'ro');
    pause(0.001);
    %disp(curState);
    
    oldError = error;
    error = norm(curState(1:2) - goal(1:2));
    disp(error);
    if error < 0.1 || oldError <= 0.1 * error
        break;
    end
    t = t + dt;
  
end

    function x_base_next = dynamics_mm(u, x_base, dt, r, L)
    x_base_next = [x_base(1) + r/2 * (u(1) + u(2)) * cos(x_base(3)) * dt;
        x_base(2) + r/2 * (u(1) + u(2)) * sin(x_base(3)) * dt;
        x_base(3) + r/L * (u(1) - u(2)) * dt];
end


