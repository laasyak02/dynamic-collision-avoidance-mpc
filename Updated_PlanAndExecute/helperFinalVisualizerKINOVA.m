% Copyright 2020 The MathWorks, Inc.

% Visualization time step
visTimeStep = 0.2;

% Low-fidelity robot model  
motionModel = jointSpaceMotionModel('RigidBodyTree', robot);

% Control robot to target trajectory points in simulation using low-fidelity model
initState = [positions(:,1);velocities(:,1)];
targetStates = [positions;velocities;accelerations]';    
[t,robotStates] = ode15s(@(t,state) helperTimeBasedStateInputsKINOVA(motionModel, timestamp, targetStates, t, state), [timestamp(1):visTimeStep:timestamp(end)], initState);

implementation_error_poses= zeros(4, 4, 0);
k=1;
imp_disp_r = [];
imp_disp_p = [];
% Visualize simulation
for j=1:size(robotStates,1)
    show(robot, robotStates(j,1:numJoints)','PreservePlot', false, 'Frames', 'off');
    poseNow = getTransform(robot, robotStates(j,1:numJoints)', endEffector);
    if t(j)~=0 & mod(t(j),1.0000)==0 
        display(t(j))
        %implementation_error_poses(:,:,end+1)=poses(:,:,k)-poseNow;
        %disp(implementation_error_poses);
        %% Implementation error i.e between nmpc computed values and actual values being implemented during simulation
        %Rotational error
        imp_rotation_err = poseNow(1:3,1:3)*inv(poses(1:3,1:3,k));
        imp_rotation_err = acos((trace(imp_rotation_err) - 1)/2);
        imp_rotation_err = rad2deg(imp_rotation_err);
        %storing error at each timestep
        imp_disp_r(end+1) = imp_rotation_err;
        
        %Position error - based on euclidean distance
        imp_position_err = sqrt(sum((poseNow(1:3,4) - poses(1:3,4,k)).^2,1));
        %storing error at each timestep
        imp_disp_p(end+1) = imp_position_err;
        k=k+1;
    end
    plot3(poseNow(1,4), poseNow(2,4), poseNow(3,4),'b.','MarkerSize',20)
    if isMovingObst
        tNow = t(j);
        aPoseObsNow = interp1(timeObs,aPosesObs,tNow);
        aMovObs.Pose = trvec2tform(aPoseObsNow);
        h1.Matrix = aMovObs.Pose;
        bPoseObsNow = interp1(timeObs,bPosesObs,tNow);
        bMovObs.Pose = trvec2tform(bPoseObsNow);
        h2.Matrix = bMovObs.Pose;
        cPoseObsNow = interp1(timeObs,cPosesObs,tNow);
        cMovObs.Pose = trvec2tform(cPoseObsNow);
        h3.Matrix = cMovObs.Pose;
    end
    drawnow;
end