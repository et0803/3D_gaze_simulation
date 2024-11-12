function [pupilImagingResult] = drawPupilImagingBasedOnRayTracing(ax, eyeAndCameraParameters)

opticalCenter = eyeAndCameraParameters.opticalCenter;
imagePlanePoint = eyeAndCameraParameters.imagePlanePoint;
eyeBallCenter = eyeAndCameraParameters.eyeBallCenter;
pupilRadius = eyeAndCameraParameters.pupilRadius;
pupilCenter = eyeAndCameraParameters.pupilCenter;
transformedPupilPoints = eyeAndCameraParameters.transformedPupilPoints;
transformedIrisPoints = eyeAndCameraParameters.transformedIrisPoints;
transformedCorneaCenter = eyeAndCameraParameters.transformedCorneaCenter;
corneaSphereRadius = eyeAndCameraParameters.corneaSphereRadius;

% light raytracing of the pupil contour
[intersectionPointsOnInnerCorneaSphere, intersectionPointsOnOuterCorneaSphere, directIntersectionPointsOnOuterCorneaSphere] = ...
    refractedPupilEdgeRayTracing(opticalCenter,transformedPupilPoints,corneaSphereRadius(2),transformedCorneaCenter(:,2),...
    corneaSphereRadius(1),transformedCorneaCenter(:,1),1.376,1.4,1);

plot3(ax,intersectionPointsOnInnerCorneaSphere(1,:),intersectionPointsOnInnerCorneaSphere(2,:),intersectionPointsOnInnerCorneaSphere(3,:),'b-','LineWidth',1);
plot3(ax,intersectionPointsOnOuterCorneaSphere(1,:),intersectionPointsOnOuterCorneaSphere(2,:),intersectionPointsOnOuterCorneaSphere(3,:),'r-','LineWidth',1);

% light raytracing of on ECP plane spanned by the eye rotation center, the camera center, and the pupil center
eyeBallCenter2PupilCenter = (pupilCenter - eyeBallCenter(:,1))/norm(pupilCenter - eyeBallCenter(:,1));
eyeBallCenter2CameraOpticalCenter = (opticalCenter - eyeBallCenter(:,1)) / norm(opticalCenter - eyeBallCenter(:,1));
if norm(cross(eyeBallCenter2CameraOpticalCenter,eyeBallCenter2PupilCenter))<1e-10
    ECPplaneNormalDirection = eyeTrans_R(:,2);
else
    ECPplaneNormalDirection = cross(eyeBallCenter2CameraOpticalCenter,eyeBallCenter2PupilCenter)/norm(cross(eyeBallCenter2CameraOpticalCenter,eyeBallCenter2PupilCenter)); % eye center, camera center, pupil center 张成的平面ECP
end
pupilRadiusDirectionOnECP = cross(ECPplaneNormalDirection,eyeBallCenter2PupilCenter) / norm(cross(ECPplaneNormalDirection,eyeBallCenter2PupilCenter));

pupilEdgePointOnECPPlane_distalToCamera = pupilCenter + pupilRadiusDirectionOnECP * pupilRadius;
pupilEdgePointOnECPPlane_proximalToCamera = pupilCenter - pupilRadiusDirectionOnECP * pupilRadius;
pupilEdgePointOnECPAndPupilCenter = [pupilEdgePointOnECPPlane_distalToCamera,pupilEdgePointOnECPPlane_proximalToCamera, pupilCenter];

[ECPIntersectionPointsOnInnerCorneaSphere, ECPIntersectionPointsOnOuterCorneaSphere, ECPDirectIntersectionPointsOnOuterCorneaSphere] = ...
    refractedPupilEdgeRayTracing(opticalCenter,pupilEdgePointOnECPAndPupilCenter,corneaSphereRadius(2),transformedCorneaCenter(:,2),...
    corneaSphereRadius(1),transformedCorneaCenter(:,1),1.376,1.4,1);

refractedECPProjectionsOnCamera = linesPlaneIntersection(opticalCenter,ECPIntersectionPointsOnOuterCorneaSphere,imagePlanePoint,ax,0);

x = [pupilEdgePointOnECPAndPupilCenter(1,:);ECPIntersectionPointsOnInnerCorneaSphere(1,:);ECPIntersectionPointsOnOuterCorneaSphere(1,:);refractedECPProjectionsOnCamera(1,:)];
y = [pupilEdgePointOnECPAndPupilCenter(2,:);ECPIntersectionPointsOnInnerCorneaSphere(2,:);ECPIntersectionPointsOnOuterCorneaSphere(2,:);refractedECPProjectionsOnCamera(2,:)];
z = [pupilEdgePointOnECPAndPupilCenter(3,:);ECPIntersectionPointsOnInnerCorneaSphere(3,:);ECPIntersectionPointsOnOuterCorneaSphere(3,:);refractedECPProjectionsOnCamera(3,:)];
plot3(ax,x(:,3),y(:,3),z(:,3),'Color',[0 0 0],'LineWidth',1,'LineStyle','--');
plot3(ax,x(:,1:2), y(:,1:2), z(:,1:2),'k-');
x = [refractedECPProjectionsOnCamera(1,:);ones(size(refractedECPProjectionsOnCamera,2))*opticalCenter(1)];
y = [refractedECPProjectionsOnCamera(2,:);ones(size(refractedECPProjectionsOnCamera,2))*opticalCenter(2)];
z = [refractedECPProjectionsOnCamera(3,:);ones(size(refractedECPProjectionsOnCamera,2))*opticalCenter(3)];
plot3(ax,x,y,z,'Color',[0 0 0],'LineWidth',0.5,'LineStyle',':');

% draw perspective projection image of the pupil contour considering refraction
refractedPupilEdgeProjectionsOnCamera = linesPlaneIntersection(opticalCenter,intersectionPointsOnOuterCorneaSphere,imagePlanePoint,ax,0);
plot3(ax,refractedPupilEdgeProjectionsOnCamera(1,:),refractedPupilEdgeProjectionsOnCamera(2,:),refractedPupilEdgeProjectionsOnCamera(3,:),'Color',[0.8000 0.6000 0],'LineWidth',1);


% mesh the light raytracing
meshX1 = [refractedPupilEdgeProjectionsOnCamera(1,:);intersectionPointsOnOuterCorneaSphere(1,:)];
meshY1 = [refractedPupilEdgeProjectionsOnCamera(2,:);intersectionPointsOnOuterCorneaSphere(2,:)];
meshZ1 = [refractedPupilEdgeProjectionsOnCamera(3,:);intersectionPointsOnOuterCorneaSphere(3,:)];
surf(meshX1,meshY1,meshZ1,'FaceColor',[0.3010 0.7450 0.9330],'FaceAlpha',0.5,'EdgeColor','none');

meshX2 = [intersectionPointsOnInnerCorneaSphere(1,:);intersectionPointsOnOuterCorneaSphere(1,:)];
meshY2 = [intersectionPointsOnInnerCorneaSphere(2,:);intersectionPointsOnOuterCorneaSphere(2,:)];
meshZ2 = [intersectionPointsOnInnerCorneaSphere(3,:);intersectionPointsOnOuterCorneaSphere(3,:)];
surf(meshX2,meshY2,meshZ2,'FaceColor',[1 1 0],'FaceAlpha',0.5,'EdgeColor','none');

meshX3 = [intersectionPointsOnInnerCorneaSphere(1,:);transformedPupilPoints(1,:)];
meshY3 = [intersectionPointsOnInnerCorneaSphere(2,:);transformedPupilPoints(2,:)];
meshZ3 = [intersectionPointsOnInnerCorneaSphere(3,:);transformedPupilPoints(3,:)];
surf(meshX3,meshY3,meshZ3,'FaceColor',[0.5490 0.2235 0.8118],'FaceAlpha',0.5,'EdgeColor','none');

pupilImagingResult.intersectionPointsOnInnerCorneaSphere =intersectionPointsOnInnerCorneaSphere;
pupilImagingResult.intersectionPointsOnOuterCorneaSphere =intersectionPointsOnOuterCorneaSphere;
pupilImagingResult.directIntersectionPointsOnOuterCorneaSphere =directIntersectionPointsOnOuterCorneaSphere;

pupilImagingResult.ECPIntersectionPointsOnInnerCorneaSphere =ECPIntersectionPointsOnInnerCorneaSphere;
pupilImagingResult.ECPIntersectionPointsOnOuterCorneaSphere =ECPIntersectionPointsOnOuterCorneaSphere;
pupilImagingResult.ECPDirectIntersectionPointsOnOuterCorneaSphere =ECPDirectIntersectionPointsOnOuterCorneaSphere;

pupilImagingResult.refractedECPProjectionsOnCamera =refractedECPProjectionsOnCamera;
pupilImagingResult.refractedPupilEdgeProjectionsOnCamera =refractedPupilEdgeProjectionsOnCamera;

end