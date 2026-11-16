%robot = importrobot("C:\\Users\laasya\OneDrive\Documents\MATLAB\\Humanoid_urdf_2.urdf");
% [robot,importInfo] = importrobot("C:\\Users\laasya\OneDrive\Documents\MATLAB\\Humanoid_urdf.urdf");
%display(robot)
% %___ = importrobot(___,"DataFormat","column")
% %robot = importrobot(___,"DataFormat","column");
% numJoints = numel(homeConfiguration(robot));
%show(robot);
%openExample("sm/DocImportedURDFModelExample");
%showdetails(robot);
%viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","l_wrist");
robot = loadrobot("atlas")
%showdetails(robot_1);
viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","r_hand");
viztree_2 = interactiveRigidBodyTree(robot,"MarkerBodyName","l_hand");
viztree_3 = interactiveRigidBodyTree(robot,"MarkerBodyName","r_foot");
viztree_4 = interactiveRigidBodyTree(robot,"MarkerBodyName","l_foot");
viztree_5= interactiveRigidBodyTree(robot,"MarkerBodyName","head");
viztree_6 = interactiveRigidBodyTree(robot,"MarkerBodyName","utorso");
% viztree_7= interactiveRigidBodyTree(robot,"MarkerBodyName","r_hand");
% viztree_8 = interactiveRigidBodyTree(robot,"MarkerBodyName","l_foot");
y1=1.0;
y2=0.5;
diff_y=y2-y1;
instance=diff_y/30;
x1_leg=0;
x2_leg=0.5;
diff_x_leg=x2_leg-x1_leg;
instance_1=diff_x_leg/30;
timeObs=30;
for timenow=1:timeObs
    raiseLeftHand = constraintPositionTarget("l_hand","TargetPosition",[0 y1 0.5]);
    raiseRightLeg = constraintPositionTarget("r_foot","TargetPosition",[x1_leg 0 -y1]);
    viztree.Constraints = {raiseLeftHand raiseRightLeg};
    viztree_2.Constraints = {raiseLeftHand raiseRightLeg};
    viztree_3.Constraints = {raiseLeftHand raiseRightLeg};
    viztree_4.Constraints = {raiseLeftHand raiseRightLeg};
    viztree_5.Constraints = {raiseLeftHand raiseRightLeg};
    viztree_6.Constraints = {raiseLeftHand raiseRightLeg};
    y1=y1+instance;
    x1_leg=x1_leg+instance_1;
    pose_rh=viztree.MarkerBodyPose(:,4);
    pose_lh=viztree_2.MarkerBodyPose(:,4);
    pose_rf=viztree_3.MarkerBodyPose(:,4);
    pose_lf=viztree_4.MarkerBodyPose(:,4);
    pose_head=viztree_5.MarkerBodyPose(:,4);
    pose_torso=viztree_6.MarkerBodyPose(:,4);
    m=collisionMesh([pose_rh(1)+0.04,pose_rh(2),pose_rh(3);pose_rh(1)-0.04,pose_rh(2),pose_rh(3);...
        pose_lh(1)+0.04,pose_lh(2),pose_lh(3);pose_lh(1)-0.04,pose_lh(2),pose_lh(3);pose_rf(1),pose_rf(2)+0.05,...
        pose_rf(3);pose_rf(1),pose_rf(2)-0.05,pose_rf(3);pose_lf(1),pose_lf(2)+0.05,pose_lf(3);pose_lf(1),...
        pose_lf(2)-0.05,pose_lf(3);pose_head(1),pose_head(2)+0.08,pose_head(3);pose_head(1),pose_head(2)-0.08,...
        pose_head(3);pose_torso(1)+0.2,pose_torso(2),pose_torso(3);pose_torso(1)-0.2,pose_torso(2),pose_torso(3)]);
    show(m);
    pause(1);
end
% figure;
% viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","l_hand");
% m=collisionMesh([0.25,0.2,0.35;0.02,1.2,0.55;0.25,0.2,0.55;0.02,1.2,0.35;0.25,1.2,0.35;0.25,1.2,0.55;0.02,0.2,0.35;0.02,0.2,0.55]);
% show(m)
% %raiseRightLeg = constraintPositionTarget("l_hand","TargetPosition",[0 1 0.5]);
% %viztree.Constraints = {raiseRightLeg}; 
% figure;
% viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","l_hand");
% hold on;
% display(viztree.MarkerBodyPose);
% viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","r_foot");
% display(viztree.MarkerBodyPose);
%display(pose_lh(:,4));