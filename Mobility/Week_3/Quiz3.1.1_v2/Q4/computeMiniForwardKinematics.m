function [endeff] = computeMiniForwardKinematics(rads1,rads2)

L1 = 1;
L2 = 1;
alpha_world = (1/2) * (rads1 + rads2) + pi;
	beta_world = (rads1 - rads2);

	alpha_polar = alpha_world;
	r_polar = sqrt(L2^2 - (L1 * sin(beta_world)^2)) - L1 * cos(beta_world);

	x_world = r_polar * cos(alpha_polar);
	y_world = r_polar * sin(alpha_polar);

	endeff = [x_world, y_world];