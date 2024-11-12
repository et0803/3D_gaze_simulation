function [eyeParameters] = drawEyeAndCameraSystemWithParameters(ax, eyePositionInReferrenceFrame, eyeFrameReferrenceRotation, kappaCalibrationVectorInLeftEyeFrame, eyeAngleHorizontal, eyeAngleVertical, cameraRotation, cameraPosition,cameraFocalLength, visualAxisLength)
%左眼的零位参考系为全局参考系
eyeTrans_t = eyePositionInReferrenceFrame;
pupilEdgeSampleNum=100;

% draw camera
drawCameraFrameAxis = 1;
cameraTrans_R = cameraRotation;
cameraTrans_t = cameraPosition;
cameraTrans = [cameraTrans_R,cameraTrans_t;0,0,0,1];
[opticalCenter,imagePlanePoint]= drawCameraModel2(drawCameraFrameAxis,cameraFocalLength,cameraTrans,ax);

% draw eye ball
eyeTrans_R_init = eyeFrameReferrenceRotation;

currentEyeAlpha = eyeAngleHorizontal*pi/180; 
currentEyeBeta = eyeAngleVertical*pi/180;  
eyeTrans_R = eyeTrans_R_init*eul2rotm([currentEyeAlpha,currentEyeBeta,0],'XYZ');
eyeTrans = [eyeTrans_R,eyeTrans_t;0,0,0,1];

[eyeBallCenter, pupilRadius, pupilCenter, transformedPupilPoints, transformedIrisPoints, transformedCorneaCenter, corneaSphereRadius]= drawEyeBallModel(1,pupilEdgeSampleNum,1,eyeTrans,ax);

% draw eye frame.
axisLength = 0.0075;
axisLengthForRotatedEyeFrame = axisLength*0.3;
rotatedEyeFrame= eyeTrans_R*axisLengthForRotatedEyeFrame;
globalInitFrame = eyeTrans_R_init*axisLength*2;
hold on;

quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),globalInitFrame(1,1),globalInitFrame(2,1),globalInitFrame(3,1),'r-');
quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),globalInitFrame(1,2),globalInitFrame(2,2),globalInitFrame(3,2),'g-');
quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),globalInitFrame(1,3),globalInitFrame(2,3),globalInitFrame(3,3),'b-');

quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),rotatedEyeFrame(1,1),rotatedEyeFrame(2,1),rotatedEyeFrame(3,1),'r-');
quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),rotatedEyeFrame(1,2),rotatedEyeFrame(2,2),rotatedEyeFrame(3,2),'g-');
quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),rotatedEyeFrame(1,3),rotatedEyeFrame(2,3),rotatedEyeFrame(3,3),'b-');


% optical axis of the eye
hold on;
opticalAxisArrowLength = 0.05;
opticalAxisArrowEndPoint = pupilCenter + opticalAxisArrowLength*rotatedEyeFrame(:,3)/norm(rotatedEyeFrame(:,3));
opticalAxisDirection = [pupilCenter,opticalAxisArrowEndPoint];
plot3(opticalAxisDirection(1,:),opticalAxisDirection(2,:),opticalAxisDirection(3,:),'k--');
opticalAxisDirection_innerEye = [pupilCenter,eyeTrans_t];
plot3(opticalAxisDirection_innerEye(1,:),opticalAxisDirection_innerEye(2,:),opticalAxisDirection_innerEye(3,:),'k:','LineWidth',1);

% visual axis of the eye
visualAxisArrowLength = visualAxisLength;
visualAxisArrowEndPoint = (eyeBallCenter(:,2) + eyeBallCenter(:,3))/2+ visualAxisArrowLength*eyeTrans_R*kappaCalibrationVectorInLeftEyeFrame/norm(kappaCalibrationVectorInLeftEyeFrame); %起点为角膜内、外表面的球心连线的中心
visualAxisDirection = [(eyeBallCenter(:,2) + eyeBallCenter(:,3))/2, visualAxisArrowEndPoint];
plot3(visualAxisDirection(1,:),visualAxisDirection(2,:),visualAxisDirection(3,:),'k');

eyeParameters.opticalCenter = opticalCenter;
eyeParameters.imagePlanePoint =imagePlanePoint;
eyeParameters.eyeBallCenter=eyeBallCenter;
eyeParameters.pupilRadius = pupilRadius;
eyeParameters.pupilCenter = pupilCenter;
eyeParameters.transformedPupilPoints = transformedPupilPoints;
eyeParameters.transformedIrisPoints = transformedIrisPoints;
eyeParameters.transformedCorneaCenter = transformedCorneaCenter;
eyeParameters.corneaSphereRadius = corneaSphereRadius;
end