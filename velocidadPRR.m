function xdot = velocidadPRR(q, qdot)
% ============================================================
% Calcula la velocidad cartesiana del efector final de un robot PRR
% ============================================================

l1 = 108.4;
l2 = 82.5;
l3 = 85;

d1 = q(1);
theta2 = q(2);
theta3 = q(3);

% Jacobiano analítico directo (sin simbólico)
Jp = [ 0, -l2*sin(theta2) - l3*sin(theta2 + theta3), -l3*sin(theta2 + theta3);
       0,  l2*cos(theta2) + l3*cos(theta2 + theta3),  l3*cos(theta2 + theta3);
       1,  0,  0 ];

xdot = Jp * qdot(:);
end
