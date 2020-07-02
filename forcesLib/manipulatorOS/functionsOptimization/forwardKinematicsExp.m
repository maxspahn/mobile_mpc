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

