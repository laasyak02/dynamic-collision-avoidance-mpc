% Copyright 2020 The MathWorks, Inc.

% Potential static obstacles

aObs = collisionSphere(0.02);  
aObs.Pose = trvec2tform([0.2 0.2 0.25]);   

bObs = collisionSphere(0.08);  
bObs.Pose = trvec2tform([0.32 0.3 0.39]);

cObs = collisionSphere(0.05);  
cObs.Pose = trvec2tform([0.3 0.3 0.4]);  

dObs = collisionBox(0.4,0.04,0.25);  
dObs.Pose = trvec2tform([0.2 0.4 0.15]); 

% Potential moving obstacles
% Create moving obstacles
aMovObs = collisionSphere(0.04);
bMovObs = collisionSphere(0.02);
cMovObs = collisionSphere(0.05);
% Assign pre-defined trajectories to moving obstacles for 30 s in the
% future. The algorithm only has access to the poses at the current time
% step. This is for simulation purpose only and this could be readings from
% a sensor.
tsObs = 0.1;
freqMotion = 1/15;
timeObs = 0:tsObs:30; 
aPosesObs = repmat([0.4 0.35 0.35],length(timeObs),1);
aPosesObs(:,3) = 0.15 + 0.35 * cos( (2*pi*freqMotion)*timeObs' );
bPosesObs = repmat([0.4 0.5 0.15],length(timeObs),1);
bPosesObs(:,1) = 0.15 + 0.15 * sin( (2*pi*freqMotion)*timeObs' );
cPosesObs = repmat([0.6 0.25 0.25],length(timeObs),1);
cPosesObs(:,1) = 0.15 + 0.15 * sin( (2*pi*freqMotion)*timeObs' );

if isMovingObst
    % Query poses of the moving obstacles at time t = 0
    tNow = 0;
    aPoseObsNow = interp1(timeObs,aPosesObs,tNow);
    aMovObs.Pose = trvec2tform(aPoseObsNow);
    bPoseObsNow = interp1(timeObs,bPosesObs,tNow);
    bMovObs.Pose = trvec2tform(bPoseObsNow);
    cPoseObsNow = interp1(timeObs,cPosesObs,tNow);
    cMovObs.Pose = trvec2tform(cPoseObsNow);
    world = {aMovObs, bMovObs, cMovObs};
else
    % Set static obstacles
    world = {aObs, cObs}; % try adding more static obstacles aObs to dObs or create your own
end