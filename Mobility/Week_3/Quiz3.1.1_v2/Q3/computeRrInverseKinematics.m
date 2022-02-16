function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;


%L1 = 1; 
%L2 = 1;


%% Effector

L1 = 1; 
L2 = 1;

		theta_2_cos = (X^2 + Y^2 - L1^2 - L2^2) / (2 * L1 * L2);

		
		theta_2_sin_pos = sqrt(1-((X^2 + Y^2 - L1^2 - L2^2) / (2 * L1 * L2))^2);
		%theta_2_sin_neg = -(sqrt(1-((X^2 + Y^2 - L1^2 - L2^2) / (2 * L1 * L2))^2));

		theta_2_pos = atan2(theta_2_sin_pos, theta_2_cos);
		%theta_2_neg = atan2(theta_2_sin_neg, theta_2_cos);

		%% Elbow
		theta_1_cos_pos = (X * (L1 + L2 * theta_2_cos) + Y * l2 * theta_2_sin_pos) / (X^2 + Y^2);
		%theta_1_cos_neg = (X * (L1 + L2 * theta_2_cos) + Y * l2 * theta_2_sin_neg) / (X^2 + Y^2);

		theta_1_sin_pos = sqrt(1-(theta_1_cos_pos));
		%theta_1_sin_neg = sqrt(1-(theta_1_cos_neg));

		theta_1_pos = atan2(theta_1_sin_pos, theta_1_cos_pos);
		%theta_1_neg = atan2(theta_1_sin_pos, theta_1_cos_neg);

		%% Solution
		rads1=theta_1_pos;
		rads2=theta_2_pos;