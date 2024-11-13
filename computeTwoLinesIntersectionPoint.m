function [intersectionPoint] = computeTwoLinesIntersectionPoint(viaPointOfLine1, directionOfLine1, viaPointOfLine2, directionOfLine2)

% ((viaPointOfLine1 + lambda1*directionOfLine1) - (viaPointOfLine2 + lambda2*directionOfLine2)) ⊥ directionOfLine1
% ((viaPointOfLine1 + lambda1*directionOfLine1) - (viaPointOfLine2 + lambda2*directionOfLine2)) ⊥ directionOfLine2


A = [dot(directionOfLine1,directionOfLine1), -dot(directionOfLine2,directionOfLine1); 
     dot(directionOfLine1,directionOfLine2), -dot(directionOfLine2,directionOfLine2)];

b = [-dot((viaPointOfLine1-viaPointOfLine2),directionOfLine1); -dot((viaPointOfLine1-viaPointOfLine2),directionOfLine2)];

lambda = A\b;

intersectionPoint = ((viaPointOfLine1+lambda(1)*directionOfLine1)+(viaPointOfLine2+lambda(2)*directionOfLine2))/2;

end