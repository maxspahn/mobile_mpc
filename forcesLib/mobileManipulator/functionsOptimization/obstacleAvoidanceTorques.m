function ineq = obstacleAvoidanceTorque(z, p)
    x = z(1:3);
    q = z(4:10);
    objects = p(21:end);
    
    spheres = computeSpheres(q, x);
    nbObstacles = size(objects, 1)/4;
    nbSpheres = size(spheres, 1);
    ineq = [];
    for i=1:nbObstacles
        for j = 1:nbSpheres
            ineq = [ineq; sqrt((spheres(j, 1) - objects(4 * (i - 1) + 1))^2 +...
                    (spheres(j, 2) - objects(4 * (i - 1) + 2))^2 +...
                    (spheres(j, 3) - objects(4 * (i - 1) + 3))^2) - objects(4 * (i - 1) + 4) - spheres(j, 4)];
        end
    end

end

function spheres = computeSpheres(q, x)
    Ts = forwardKinematicsExp(q, x);
    T_base = Ts(:,1:4);
    T0 = Ts(:,5:8);
    T1 = Ts(:,9:12);
    T2 = Ts(:,13:16);
    T3 = Ts(:,17:20);
    T4 = Ts(:,21:24);
    T5 = Ts(:,25:28);
    T6 = Ts(:,29:32);
    T7 = Ts(:,33:36);
    T8 = Ts(:,37:40);
    
    hbase = [0.25 0 0.2];
    rbase = 0.5;

    Tbase_s = T_base * makeTF('translate', hbase);
    s_base = [Tbase_s(1:3,4)', rbase];

    %% link0
    l1 = [0 0 1.2 * 0.333/2];
    T0_s = T_base * T0 * makeTF('translate', l1);
    s_0 = [T0_s(1:3, 4)', l1(3)];


    %% link2
    l2 = [0 1.2 * -0.3160/2 0];
    T2_s = T_base * T0 * T1 * T2 * makeTF('translate', l2);
    s_2 = [T2_s(1:3, 4)', -l2(2)];


    %% link 3
    l3 = [0 0 1.2 * 0.384/2];
    T3_s = T_base * T0 * T1 * T2 * T3 * makeTF('translate', l3);
    s_3 = [T3_s(1:3, 4)', l3(3)];

    %% link 4
    l4 = [1.2 * 0.0826/2 0 1,2 * 0.384/2];
    T4_s = T_base * T0 * T1 * T2 * T3 * T4 * makeTF('translate', l3);
    s_4 = [T4_s(1:3, 4)', l4(3)];


    %% link EE
    ree = 0.2;
    Tee_s = T_base * T0 * T1 * T2 * T3 * T4 * T5 * T6 * T7 ;
    s_ee = [Tee_s(1:3, 4)', ree];
    
    spheres = [s_base; s_0; s_2; s_3; s_4; s_ee];
end

function Ts = forwardKinematicsExp(q, x)

    T_base = makeTF('translate', [x(1), x(2), 0]) * makeTF('zrotate', x(3));
    T0 = makeTF('translate', [0 0 0.5995]);
    T1 = makeTF('translate', [0, 0, 0.3330]) * makeTF('zrotate', q(1));
    T2 = makeTF('xrotate', -pi/2) * makeTF('zrotate', q(2));
    T3 = makeTF('xrotate', pi/2) * makeTF('translate', [0 0 0.3160]) * makeTF('zrotate', q(3));
    T4 = makeTF('xrotate', pi/2) * makeTF('translate', [0.0826 0 0]) * makeTF('zrotate', q(4));
    T5 = makeTF('xrotate', -pi/2) * makeTF('translate', [-0.0826, 0 0.3840]) * makeTF('zrotate', q(5));
    T6 = makeTF('xrotate', pi/2) * makeTF('zrotate', q(6));
    T7 = makeTF('xrotate', pi/2) * makeTF('translate', [0.0881 0 0]) * makeTF('zrotate', q(7));
    T8 = makeTF('translate', [0 0 0.107]);

    Ts = [T_base, T0, T1, T2, T3, T4, T5, T6, T7, T8];

end

function T = makeTF(type, input)

    if strcmp(type, 'translate')
        T = [1 0 0 input(1); 0 1 0 input(2); 0 0 1 input(3); 0 0 0 1];
    elseif strcmp(type, 'xrotate')
        T = [1 0 0 0; 0 cos(input) -sin(input) 0; 0 sin(input) cos(input) 0; 0 0 0 1];
    elseif strcmp(type, 'yrotate')
        T = [cos(input) 0 sin(input) 0; 0 1 0 0; -sin(input) 0 cos(input) 0; 0 0 0 1];
    elseif strcmp(type, 'zrotate')
        T = [cos(input) -sin(input) 0 0; sin(input) cos(input) 0 0; 0 0 1 0; 0 0 0 1];
    end
end

