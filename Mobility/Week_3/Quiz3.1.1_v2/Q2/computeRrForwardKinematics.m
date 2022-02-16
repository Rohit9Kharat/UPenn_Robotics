function [elbow,endeff] = computeRrForwardKinematics(rads1,rads2)
%%GIVEN THE ANGLES OF THE MOTORS, return an array of arrays for the
%%position of the elbow [x1,y1], and endeffector [x2,y2]

L1 = 1;
L2 = 1;
theta1 = rads1;
theta2 = rads2;


elbow = [L1*cos(theta1),L1*sin(theta1)];
endeff =[L1*cos(theta1)+ (L2*cos(theta1 + theta2)),L1*sin(theta1)+ (L2*sin(theta1 + theta2))];
