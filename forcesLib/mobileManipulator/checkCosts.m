function costs = checkCosts(problem, output)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
fn = fieldnames(output);
parameters = problem.all_parameters(1:200);
for k=1:numel(fn)
    if (isnumeric(output.(fn{k})) )
        c = costFunctionSimple(output.(fn{k}), parameters);
        ineq = obstacleAvoidanceSimple(output.(fn{k}), parameters);
        disp(c);
        fprintf("Ineq : %1.10f\n", ineq);
    end
end
    
end

