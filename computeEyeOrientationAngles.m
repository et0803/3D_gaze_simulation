function [angle1OfEulerXYZ, angle2OfEulerXYZ] = computeEyeOrientationAngles(gazePointInEyeInitReferrenceFrame,kappaCalibrationVecterInEyeFrame, intersectionOfVisualAxisAndOpticalAxisOffsetInEyeFrame)
% compute orientation angle of the two eyes
% euler angle 'XYZ'，视轴起点近似为眼球旋转中心[0,0,0]。在眼球参考坐标系中计算
% xyz欧拉角顺序下，假设眼球绕z轴的自旋角为0，则
% R = [ c2       0      s2    ;
%       s1s2     c1    -s1c2  ;
%      -c1s2     s1     c1c2  ;]
% R * normalizedLeftKappaCalibrationVecterInLeftEyeFrame = leftNormalizedGazeInLeftEyeInitReferrenceFrame

normalizedGazeDirectionInEyeInitReferrenceFrame = gazePointInEyeInitReferrenceFrame/norm(gazePointInEyeInitReferrenceFrame);
normalizedKappaCalibrationVecterInEyeFrame = kappaCalibrationVecterInEyeFrame/norm(kappaCalibrationVecterInEyeFrame);

% 左眼, 在眼球参考坐标系内，标准化的凝视方向向量的第一个分量为眼球姿态矩阵第一行 [c2 0 s2] 和 标准化的KappaCalibrationVecter的内积。
referenceAngleForKappa = atan2(normalizedKappaCalibrationVecterInEyeFrame(3), normalizedKappaCalibrationVecterInEyeFrame(1));
angleDistance2 = acos(normalizedGazeDirectionInEyeInitReferrenceFrame(1)/norm(normalizedKappaCalibrationVecterInEyeFrame([1,3])));
if abs(referenceAngleForKappa - angleDistance2) < pi/2
    eyeOrientationAngle2 = referenceAngleForKappa - angleDistance2;
end
if abs(referenceAngleForKappa + angleDistance2) < pi/2
    eyeOrientationAngle2 = referenceAngleForKappa + angleDistance2;
end

% 左眼，在眼球参考坐标系内，标准化的凝视方向向量的第二个分量为眼球姿态矩阵第一行 [s1s2  c1  -s1c2] 和 标准化的KappaCalibrationVecter的内积。 转换成 [c1  s1] * rearrangedVector
rearrangedVector = [normalizedKappaCalibrationVecterInEyeFrame(2), normalizedKappaCalibrationVecterInEyeFrame(1)*sin(eyeOrientationAngle2) - normalizedKappaCalibrationVecterInEyeFrame(3)*cos(eyeOrientationAngle2)];
referenceAngleForRearrangedVector = atan2(rearrangedVector(2), rearrangedVector(1));
angleDistance1 = acos(normalizedGazeDirectionInEyeInitReferrenceFrame(2)/norm(rearrangedVector));

if abs(referenceAngleForRearrangedVector - angleDistance1) < pi/2
    eyeOrientationAngle1 = referenceAngleForRearrangedVector - angleDistance1;
end
if abs(referenceAngleForRearrangedVector + angleDistance1) < pi/2
    eyeOrientationAngle1 = referenceAngleForRearrangedVector + angleDistance1;
end

% 检验计算结果
rotationMatrixFromEyeFrameToEyeInitFrame = eul2rotm([eyeOrientationAngle1,eyeOrientationAngle2,0],'XYZ');
calculatedGazeDirectionInEyeInitFrame = rotationMatrixFromEyeFrameToEyeInitFrame *  kappaCalibrationVecterInEyeFrame/norm(kappaCalibrationVecterInEyeFrame);
actualGazeDirectionInEyeInitFrame = gazePointInEyeInitReferrenceFrame - rotationMatrixFromEyeFrameToEyeInitFrame*[0;0;intersectionOfVisualAxisAndOpticalAxisOffsetInEyeFrame];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 进一步优化求解
% R * normalizedLeftKappaCalibrationVecterInLeftEyeFrame ∥ (leftGazePointInLeftEyeInitReferrenceFrame - R * intersectionOfVisualAxisAndOpticalAxisInEyeFrame)

while acos(dot(calculatedGazeDirectionInEyeInitFrame, actualGazeDirectionInEyeInitFrame)/(norm(calculatedGazeDirectionInEyeInitFrame)*norm(actualGazeDirectionInEyeInitFrame)))/pi*180 >1e-8
    % 调整凝视方向，重复上面的求解过程
    updatedGazeDirectionInEyeInitReferenceFrame = gazePointInEyeInitReferrenceFrame - rotationMatrixFromEyeFrameToEyeInitFrame*[0;0;intersectionOfVisualAxisAndOpticalAxisOffsetInEyeFrame];

    normalizedGazeDirectionInEyeInitReferrenceFrame = updatedGazeDirectionInEyeInitReferenceFrame/norm(updatedGazeDirectionInEyeInitReferenceFrame);
    normalizedKappaCalibrationVecterInEyeFrame = kappaCalibrationVecterInEyeFrame/norm(kappaCalibrationVecterInEyeFrame);

    % 左眼, 在眼球参考坐标系内，标准化的凝视方向向量的第一个分量为眼球姿态矩阵第一行 [c2 0 s2] 和 标准化的KappaCalibrationVecter的内积。
    referenceAngleForKappa = atan2(normalizedKappaCalibrationVecterInEyeFrame(3), normalizedKappaCalibrationVecterInEyeFrame(1));
    angleDistance2 = acos(normalizedGazeDirectionInEyeInitReferrenceFrame(1)/norm(normalizedKappaCalibrationVecterInEyeFrame([1,3])));
    if abs(referenceAngleForKappa - angleDistance2) < pi/2
        eyeOrientationAngle2 = referenceAngleForKappa - angleDistance2;
    end
    if abs(referenceAngleForKappa + angleDistance2) < pi/2
        eyeOrientationAngle2 = referenceAngleForKappa + angleDistance2;
    end

    % 左眼，在眼球参考坐标系内，标准化的凝视方向向量的第二个分量为眼球姿态矩阵第一行 [s1s2  c1  -s1c2] 和 标准化的KappaCalibrationVecter的内积。 转换成 [c1  s1] * rearrangedVector
    rearrangedVector = [normalizedKappaCalibrationVecterInEyeFrame(2), normalizedKappaCalibrationVecterInEyeFrame(1)*sin(eyeOrientationAngle2) - normalizedKappaCalibrationVecterInEyeFrame(3)*cos(eyeOrientationAngle2)];
    referenceAngleForRearrangedVector = atan2(rearrangedVector(2), rearrangedVector(1));
    angleDistance1 = acos(normalizedGazeDirectionInEyeInitReferrenceFrame(2)/norm(rearrangedVector));

    if abs(referenceAngleForRearrangedVector - angleDistance1) < pi/2
        eyeOrientationAngle1 = referenceAngleForRearrangedVector - angleDistance1;
    end
    if abs(referenceAngleForRearrangedVector + angleDistance1) < pi/2
        eyeOrientationAngle1 = referenceAngleForRearrangedVector + angleDistance1;
    end


    rotationMatrixFromEyeFrameToEyeInitFrame = eul2rotm([eyeOrientationAngle1,eyeOrientationAngle2,0],'XYZ');
    calculatedGazeDirectionInEyeInitFrame = rotationMatrixFromEyeFrameToEyeInitFrame *  kappaCalibrationVecterInEyeFrame/norm(kappaCalibrationVecterInEyeFrame);
    actualGazeDirectionInEyeInitFrame = gazePointInEyeInitReferrenceFrame - rotationMatrixFromEyeFrameToEyeInitFrame*[0;0;intersectionOfVisualAxisAndOpticalAxisOffsetInEyeFrame];
end

angle1OfEulerXYZ = eyeOrientationAngle1;
angle2OfEulerXYZ = eyeOrientationAngle2;

end