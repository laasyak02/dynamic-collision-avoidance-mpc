% Copyright 2020 The MathWorks, Inc.

tolerance = 0.03;
jointTempFinal = info.Xopt(2,1:numJoints);
taskTempFinal = getTransform(robot, jointTempFinal', endEffector);
poses(:,:,end+1)=taskTempFinal;
anglesTempFinal = rotm2eul(taskTempFinal(1:3,1:3), 'XYZ');
poseTempFinal =  [taskTempFinal(1:3,4);anglesTempFinal'];
diffTerminal = abs([poseFinal(1:3)-poseTempFinal(1:3); angdiff(poseTempFinal(4:6),poseFinal(4:6))]);
flag = ones(size(Qt,1),1) & diag(Qt);
if all(diffTerminal(flag)<tolerance)
    disp('Target configuration reached.')

    goalReached = true;
    %% End Effector final position error
    %% Position
    nlmpc_error_p = diffTerminal(1:3).^2;
    nlmpc_error_p = sum(nlmpc_error_p)/3;
    nlmpc_error_p = nlmpc_error_p^0.5;
    display(nlmpc_error_p);
    %% Rotation
    nlmpc_error_o = diffTerminal(4:6).^2;
    nlmpc_error_o = sum(nlmpc_error_o)/3;
    nlmpc_error_o = nlmpc_error_o^0.5;
    display(nlmpc_error_o);
        

end