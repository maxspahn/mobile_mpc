function ineq = obstacleAvoidanceSphere_7(z, p, pMap)
x = z(1:3);
q = z(4:10);
slack = z(11);

t = 7;
if (t < 5)
  passedTime = p(pMap.dt1) * t;
else
  passedTime = p(pMap.dt1) * 4 + p(pMap.dt2) * (t - 4);
end

safetyMarginBase = p(pMap.safetyMarginBase);
safetyMarginArm = p(pMap.safetyMarginArm);

movingObstacles = p(pMap.movingObstacles(1):pMap.movingObstacles(2));
staticSpheres = p(pMap.staticSpheres(1):pMap.staticSpheres(2));

dimMovingObstacle = pMap.dimMovingObstacle;
nbMovingObstacles = pMap.nbMovingObstacles;
nbStaticSpheres = pMap.nbStaticSpheres;
dimStaticSphere = pMap.dimStaticSphere;


    
    %spheres = p(20:23);
spheres = computeSpheres(q, x)';
nbSpheres = size(spheres, 1)/4;
ineq = [];
    
    
    for j=1:nbSpheres
        s = spheres(4 * (j - 1) + 1:4 * (j - 1) + 4);
        for i=1:nbStaticSpheres
            sS = staticSpheres(dimStaticSphere * (i -1) + 1: dimStaticSphere * i);
            dist = point2point(sS(1:3), s(1:3));
            if (j < 3)
                ineq = [ineq; dist - s(4) - sS(4) - safetyMarginBase + slack];
            else
                ineq = [ineq; dist - s(4) - sS(4)- safetyMarginArm + slack];
            end
        end
    end
    
    for j=1:nbSpheres
        s = spheres(4 * (j - 1) + 1:4 * (j - 1) + 4);
        for i=1:nbMovingObstacles
            o = movingObstacles(dimMovingObstacle * (i -1) + 1: dimMovingObstacle * i);
            expectedPosition = o(1:3) + o(4:6) * passedTime;
            dist = point2point(expectedPosition(1:3), s(1:3));
            if (j < 3)
                ineq = [ineq; dist - s(4) - o(7) - safetyMarginBase + slack];
            else
                ineq = [ineq; dist - s(4) - o(7)- safetyMarginArm + slack];
            end
        end
    end
    
            
    
    % Self Collision Avoidance between end effector and base
%     ee = nbSpheres - 1;
%     ee_sphere = spheres(4 * (nbSpheres - 1) + 1:(4 * (nbSpheres - 1) + 4));
%     dist = sqrt((spheres(1) - ee_sphere(1))^2 + (spheres(2) - ee_sphere(2))^2 + (spheres(3) - ee_sphere(3))^2);
%     ineq = [ineq; dist - spheres(4) - spheres(4 * (ee) + 4) + slack];
    
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
    
    hbase1 = [0.0 0 0.25];
    rbase1 = 0.3;

    Tbase_s1 = T_base * makeTF('translate', hbase1);
    s_base1 = [Tbase_s1(1:3,4)', rbase1];
    
    hbase2 = [0.3 0 0.25];
    rbase2 = 0.3;

    Tbase_s2 = T_base * makeTF('translate', hbase2);
    s_base2 = [Tbase_s2(1:3,4)', rbase2];

    %% link0
    l1 = [0 0 1.2 * 0.333/2];
    T0_s = T_base * T0 * makeTF('translate', l1);
    s_0 = [T0_s(1:3, 4)', l1(3)];


    %% link2
    l2 = [0 1.2 * -0.3160/2 0];
    T2_s = T_base * T0 * T1 * T2 * makeTF('translate', l2);
    s_2 = [T2_s(1:3, 4)', -1.2 * l2(2)];


    %% link 3
    l3 = [0 0 1.2 * 0.384/2];
    T3_s = T_base * T0 * T1 * T2 * T3 * makeTF('translate', l3);
    s_3 = [T3_s(1:3, 4)', l3(3)];

    %% link 4
    l4 = [1.2 * 0.0826/2 0 1.2 * 0.384/2];
    T4_s = T_base * T0 * T1 * T2 * T3 * T4 * makeTF('translate', l3);
    s_4 = [T4_s(1:3, 4)', l4(3)];


    %% link EE
    ree = 0.3;
    Tee_s = T_base * T0 * T1 * T2 * T3 * T4 * T5 * T6 * T7 ;
    s_ee = [Tee_s(1:3, 4)', ree];
    
    spheres = [s_base1, s_base2, s_0, s_2, s_3, s_4, s_ee];
    spheres = [s_base1, s_base2, s_2, s_ee];
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

