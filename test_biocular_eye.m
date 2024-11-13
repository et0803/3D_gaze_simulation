%% clear the env
clc;
clear;

%% parameters of left eye and camera
leftEyePositionInGlobalReferrenceFrame = [0;0;0];
leftEyeInitReferrenceFrameToGlobalReferrenceFrame = eul2rotm([pi/2,0,pi/2],'XYZ');
leftKappaCalibrationVecterInLeftEyeFrame = [0.001;0.0875;1];  % kappa angle of 5 degree

leftCameraFrameToLeftEyeInitReferrenceFrame = eul2rotm([pi,pi/6,0],'XYZ');
leftCameraCenterInLeftEyeInitReferrenceFrame = [-0.02;0;0.045];
leftCameraTrans_R = leftEyeInitReferrenceFrameToGlobalReferrenceFrame*leftCameraFrameToLeftEyeInitReferrenceFrame;
leftCameraTrans_t = leftEyeInitReferrenceFrameToGlobalReferrenceFrame*leftCameraCenterInLeftEyeInitReferrenceFrame + leftEyePositionInGlobalReferrenceFrame;
leftCameraFocalLength = 0.0077;

% parameters of right eye and camera
rightEyePositionInGlobalReferrenceFrame = [-0.066;0;0];
rightEyeInitReferrenceFrameToGlobalReferrenceFrame = eul2rotm([pi/2,0,pi/2],'XYZ');
rightKappaCalibrationVecterInRightEyeFrame = [0.001;-0.0875;1]; % kappa angle of 5 degree

rightCameraFrameToRightEyeInitReferrenceFrame = eul2rotm([pi,pi/6,0],'XYZ');
rightCameraCenterInRightEyeInitReferrenceFrame = [-0.02;0;0.045];
rightCameraTrans_R = rightEyeInitReferrenceFrameToGlobalReferrenceFrame*rightCameraFrameToRightEyeInitReferrenceFrame;
rightCameraTrans_t = rightEyeInitReferrenceFrameToGlobalReferrenceFrame*rightCameraCenterInRightEyeInitReferrenceFrame + rightEyePositionInGlobalReferrenceFrame;
rightCameraFocalLength = 0.0077;

baselineOfTwoEyes = [leftEyePositionInGlobalReferrenceFrame, rightEyePositionInGlobalReferrenceFrame];
visualAxisLength = 0.01;


% draw two eyes with ray tracing show test
figure(1);
ax =axes();
xlabel('x');
ylabel('y');
zlabel('z');

leftEyeAngleHorizontal = -30;
leftEyeAngleVertical = 20;
leftEyeParameters = drawEyeAndCameraSystemWithParameters(ax, leftEyePositionInGlobalReferrenceFrame, leftEyeInitReferrenceFrameToGlobalReferrenceFrame, leftKappaCalibrationVecterInLeftEyeFrame, ...
                                                         leftEyeAngleHorizontal, leftEyeAngleVertical, leftCameraTrans_R, leftCameraTrans_t,leftCameraFocalLength,visualAxisLength);
drawPupilImagingBasedOnRayTracing(ax, leftEyeParameters);

rightEyeAngleHorizontal = 30;
rightEyeAngleVertical = 20;
rightEyeParameters = drawEyeAndCameraSystemWithParameters(ax, rightEyePositionInGlobalReferrenceFrame, rightEyeInitReferrenceFrameToGlobalReferrenceFrame, rightKappaCalibrationVecterInRightEyeFrame, ...
                                                          rightEyeAngleHorizontal, rightEyeAngleVertical, rightCameraTrans_R, rightCameraTrans_t,rightCameraFocalLength, visualAxisLength);
drawPupilImagingBasedOnRayTracing(ax, rightEyeParameters);
plot3(baselineOfTwoEyes(1,:),baselineOfTwoEyes(2,:),baselineOfTwoEyes(3,:),'k--');

%% 生成凝视点坐标
figure(1);
ax =axes();
xlabel('x');
ylabel('y');
zlabel('z');
intersecionsOfVisualAxisInLeftEyeInitReferrenceFrame = zeros([3,100]);
depth = 0.6;
for i=1:10
    for j = 1:10
        intersectionInLeftEyeInitReferrenceFrame = [i*0.1-0.5; j*0.1-0.5; depth];
        intersecionsOfVisualAxisInLeftEyeInitReferrenceFrame(:,10*(i-1)+j) = intersectionInLeftEyeInitReferrenceFrame;
    end
end
intersecionsOfVisualAxisInGlobalReferrenceFrame = leftEyeInitReferrenceFrameToGlobalReferrenceFrame*intersecionsOfVisualAxisInLeftEyeInitReferrenceFrame;
intersecionsOfVisualAxisInRightEyeInitReferrenceFrame = rightEyeInitReferrenceFrameToGlobalReferrenceFrame'*(intersecionsOfVisualAxisInGlobalReferrenceFrame-rightEyePositionInGlobalReferrenceFrame);

plot3(intersecionsOfVisualAxisInGlobalReferrenceFrame(1,:),intersecionsOfVisualAxisInGlobalReferrenceFrame(2,:),intersecionsOfVisualAxisInGlobalReferrenceFrame(3,:),'ko');

%% 凝视方向起点近似为眼球旋转中心，根据凝视方向求解眼球姿态
leftEyeAngles = zeros([2,size(intersecionsOfVisualAxisInLeftEyeInitReferrenceFrame,2)]);
rightEyeAngles = zeros([2,size(intersecionsOfVisualAxisInLeftEyeInitReferrenceFrame,2)]);

for i=1:size(intersecionsOfVisualAxisInLeftEyeInitReferrenceFrame,2)
    [leftEyeAngle1, leftEyeAngle2] = computeEyeOrientationAngles(intersecionsOfVisualAxisInLeftEyeInitReferrenceFrame(:,i), leftKappaCalibrationVecterInLeftEyeFrame);
    [rightEyeAngle1, rightEyeAngle2] = computeEyeOrientationAngles(intersecionsOfVisualAxisInRightEyeInitReferrenceFrame(:,i), rightKappaCalibrationVecterInRightEyeFrame);

    leftEyeAngles(:,i) = [leftEyeAngle1; leftEyeAngle2];
    rightEyeAngles(:,i) = [rightEyeAngle1; rightEyeAngle2];

end

%% 测试根据视轴反向求解眼球姿态的结果
i=25;
leftEyeParameters = drawEyeAndCameraSystemWithParameters(ax, leftEyePositionInGlobalReferrenceFrame, leftEyeInitReferrenceFrameToGlobalReferrenceFrame, leftKappaCalibrationVecterInLeftEyeFrame, ...
                                                         leftEyeAngles(1,i)/pi*180,  leftEyeAngles(2,i)/pi*180, leftCameraTrans_R, leftCameraTrans_t,leftCameraFocalLength, norm(intersecionsOfVisualAxisInLeftEyeInitReferrenceFrame(:,i)));
rightEyeParameters = drawEyeAndCameraSystemWithParameters(ax, rightEyePositionInGlobalReferrenceFrame, rightEyeInitReferrenceFrameToGlobalReferrenceFrame, rightKappaCalibrationVecterInRightEyeFrame, ...
                                                          rightEyeAngles(1,i)/pi*180, rightEyeAngles(2,i)/pi*180, rightCameraTrans_R, rightCameraTrans_t,rightCameraFocalLength, norm(intersecionsOfVisualAxisInRightEyeInitReferrenceFrame(:,i)));

leftVisualAxisViaPointInGlobalReferrenceFrame = (leftEyeParameters.eyeBallCenter(:,2)+leftEyeParameters.eyeBallCenter(:,3))/2;  %角膜内表面和外表面球面的球心坐标的平均值, eyeBallCenter中有3个center，及眼球旋转中心，角膜外表面球面中心，角膜内表面球面中心。
rightVisualAxisViaPointInGlobalReferrenceFrame = (rightEyeParameters.eyeBallCenter(:,2)+rightEyeParameters.eyeBallCenter(:,3))/2;

leftVisualAxisDirectionInGlobalReferenceFrame = leftEyeInitReferrenceFrameToGlobalReferrenceFrame * eul2rotm([leftEyeAngles(1,i),leftEyeAngles(2,i),0],'XYZ')*leftKappaCalibrationVecterInLeftEyeFrame;
rightVisualAxisDirectionInGlobalReferenceFrame = rightEyeInitReferrenceFrameToGlobalReferrenceFrame * eul2rotm([rightEyeAngles(1,i),rightEyeAngles(2,i),0],'XYZ')*rightKappaCalibrationVecterInRightEyeFrame;

visualAxisIntersectionPointInGlobalReferenceFrame = computeTwoLinesIntersectionPoint(leftVisualAxisViaPointInGlobalReferrenceFrame,leftVisualAxisDirectionInGlobalReferenceFrame, rightVisualAxisViaPointInGlobalReferrenceFrame,rightVisualAxisDirectionInGlobalReferenceFrame);


