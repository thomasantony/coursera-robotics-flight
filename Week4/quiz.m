% Question 3
syms phi theta

t = [sind(30)*cosd(45); sind(30)*sind(45); cosd(30)];
psi_des = 45*pi/180;
% R = p;

psi = psi_des;
cphi = cos(phi);
ctheta = cos(theta);
cpsi = cos(psi);
sphi = sin(phi);
stheta = sin(theta);
spsi = sin(psi);

Rdes = [cpsi*ctheta - sphi*spsi*stheta, -cphi*spsi, cpsi*stheta + ctheta*sphi*spsi;
        spsi*ctheta + sphi*cpsi*stheta,  cphi*cpsi, spsi*stheta - ctheta*sphi*cpsi;
                          -cphi*stheta,       sphi,                    cphi*ctheta;];

[theta_sol, phi_sol] = solve(Rdes*[0;0;1] - t/norm(t), [theta, phi]);

format short;
Rdes_1 = double(subs(Rdes, [theta, phi], [theta_sol(1), phi_sol(1)]));
Rdes_2 = double(subs(Rdes, [theta, phi], [theta_sol(2), phi_sol(2)]));

disp(Rdes_1);
disp(Rdes_2);
% Answer is Rdes_2

% Question 3
R = [0.7244, 0.1294, 0.6771; 0.6424, -0.483, -0.595; 0.25, 0.866, -0.433];
Rdes = [0 0 1; 1 0 0; 0 1 0];

dR = R'*Rdes;
disp(dR);