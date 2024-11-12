function [opticalCenter,imagePlanePoint] = drawCameraModel2(showAxis,folcalLength,transformationMatrix,h_ax)
%绘制camera的3D模型

hold on;
scale = 1;
windowSize = [720;1080]/1080*folcalLength*scale;
cameraWindow_p1 = [-1;1;1].*[windowSize;folcalLength]; %top left
cameraWindow_p2 = [1;1;1].*[windowSize;folcalLength]; %top right
cameraWindow_p3 = [1;-1;1].*[windowSize;folcalLength]; %bottom right
cameraWindow_p4 = [-1;-1;1].*[windowSize;folcalLength]; %bottom left

cameraWindow = [cameraWindow_p1,cameraWindow_p2,cameraWindow_p3,cameraWindow_p4,cameraWindow_p1];
cameraWindow = transformationMatrix(1:3,1:3)*cameraWindow + transformationMatrix(1:3,4)* ones(1,size(cameraWindow,2));
fill3(cameraWindow(1,:),cameraWindow(2,:),cameraWindow(3,:),[1,1,1],'FaceAlpha',0.2,'FaceColor',[0.5 0.5 0.5],'LineStyle','none')

%画坐标系
axisLength = folcalLength*0.5;
opticalCenter=[0;0;0];
xEnd = [axisLength;0;0];
yEnd = [0;axisLength;0];
zEnd = [0;0;axisLength];
frameArrowPoint = [opticalCenter,xEnd,yEnd,zEnd];
frameArrowPoint = transformationMatrix(1:3,1:3)*frameArrowPoint + transformationMatrix(1:3,4)* [1,zeros(1,size(frameArrowPoint,2)-1)];

if showAxis>0
    quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,2),frameArrowPoint(2,2),frameArrowPoint(3,2),'r-');
    quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,3),frameArrowPoint(2,3),frameArrowPoint(3,3),'g-');
    quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,4),frameArrowPoint(2,4),frameArrowPoint(3,4),'b-');
end
opticalCenter = frameArrowPoint(:,1);
imagePlanePoint = cameraWindow(:,1:4);

axis equal;
hold off;
end

