% func_getJacobian.m
% depending on the _calcFlag_ this function either calculates the Jacobian
% using MATLAB-Code or calls RBDL-wrapper-function getJacobian.
% Jacobian relative to the world system
%
% WARNING: RBDL-wrapper-function getJacobian only uses the translatory
% coordinates of _T_linkSystem_desiredFrame_, ignoring the rotational part!
function J = func_getJacobian(robot,q,bodyID,T_linkSystem_desiredFrame,calcFlag)        %#eml

if (strcmp(calcFlag,'MATLAB'))    

    screwSolution = 1;
    bruteSolution = 0;
    
    %tic
    
    noJoints = robot.NoDofs;
    
    % Joint axes: position and direction vectors
    v = NaN(3,1,noJoints);
    p = NaN(3,1,noJoints);
       
    T_0_bodyID = robot.T_0_base;
    
    % Body parent chain 
    parentChain = func_getParentChain(robot, bodyID);
    
    % Save computations for joints not on the chain "base->bodyID"
    % Parent chain start from body 0
    %for curJoint = parentChain(2:end)  %simulink doesn't like this
    for index = 2:length(parentChain)
        curJoint = parentChain(index);
        T_0_bodyID = robot.T_0_j(:,:,curJoint);
        
        % the direction of the joint axis w.r.t the global system
        % both for a revolute and a prismatic joint
        v(:,:,curJoint) = T_0_bodyID(1:3,1:3)*robot.JointAxes(curJoint,:)';
        
        % position of axis
        p(:,:,curJoint) = T_0_bodyID(1:3,4);
      
    end
    
    % apply the final transformation
    T_0_bodyID = T_0_bodyID*T_linkSystem_desiredFrame;
    
    s = T_0_bodyID(1:3,4);
    
    if screwSolution
        J = zeros(6,noJoints);
        %for curJoint = parentChain(2:end)  %simulink doesn't like this
        for index = 2:length(parentChain)
            curJoint = parentChain(index);
            if robot.JointTypes(curJoint) == 1
                % it's a revolute joint
                J(1:3,curJoint) = cross(v(:,:,curJoint),s-p(:,:,curJoint));
                J(4:6,curJoint) = v(:,:,curJoint);
            else
                % it's a prismatic joint
                J(1:3,curJoint) = v(:,:,curJoint);
                J(4:6,curJoint) = zeros(3,1);
            end
            
        end
        
        %tScrew = toc;
        %     disp(['Timing for screw solution: ',num2str(tScrew)]);
        %     disp(['Rank is ',num2str(rank(J))]);
    end
    
    
    
    % Jacobian numerically;
    
    if bruteSolution
        JStar = zeros(6,noJoints);
        
        %tic
        for curJoint = 1:noJoints
            delta_q = 0.0001;
            oneVec = zeros(size(q));
            oneVec(curJoint) = 1;
            qStar = q + delta_q*oneVec;
            TStar_ij = func_getTransformations(robot,qStar);
            
            TCurStar = eye(4);
            for curJointStar = 1:bodyID
                TCurStar = TCurStar*TStar_ij(:,:,curJointStar);
                %             vStar(:,:,curJointStar) = TCurStar(1:3,3);  % z-axis is joint axis
                %             pStar(:,:,curJointStar) = TCurStar(1:3,4);  % position of axis
            end
            TCurStar = TCurStar*T_linkSystem_desiredFrame;
            
            % interesting point:
            sStar = TCurStar(1:3,4);
            
            JStar(1:3,curJoint) = (sStar-s)/delta_q;
            
            dR = TCurStar(1:3,1:3) - T_world_bodyID(1:3,1:3);
            omega_skew = (dR/delta_q)*T_world_bodyID(1:3,1:3)';
            JStar(4:6,curJoint) = omega_skew([6,7,2]);
            
        end
        
         %tBrute = toc;
         %disp(['Timing for brute solution: ',num2str(tBrute)]);
         %disp(JStar);
    end
    
%     J_transform_world_base = zeros(6);
%     J_transform_world_base(1:3,1:3) = robot.T_world_base(1:3,1:3);
%     J_transform_world_base(4:6,4:6) = robot.T_world_base(1:3,1:3);
%     J = J_transform_world_base * J;

elseif (strcmp(calcFlag,'RBDL'))
    pointPosition = T_linkSystem_desiredFrame(1:3,4);
    J = getJacobian_RBDL(robot,q,bodyID,pointPosition);
    
elseif (strcmp(calcFlag, 'MEX'))
    qDot = zeros(robot.NoDofs,1);
    qDotDot = qDot;
    pointPosition = T_linkSystem_desiredFrame(1:3,4);
    J = func_getJacobian_RBDL(robot.Name, q, qDot, qDotDot, bodyID, pointPosition);
end

