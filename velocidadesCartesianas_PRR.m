function xdot = velocidadesCartesianas_PRR(q, qdot, l2, l3)
% ============================================================
% Calcula las velocidades cartesianas (ẋ, ẏ, ż) del efector final
% de un robot 3DOF tipo PRR, a partir de las velocidades articulares.
%
% Entradas:
%   q = [d1; theta2; theta3]  -> coordenadas articulares
%   qdot = [d1dot; theta2dot; theta3dot]  -> velocidades articulares
%   l2, l3 -> longitudes de los eslabones
%
% Salida:
%   xdot = [xdot; ydot; zdot]  -> velocidades cartesianas del efector
% ============================================================

% Variables simbólicas
syms d1 theta2 theta3 real

% Cinemática directa (posición)
P = [108.4*cos(0) + l2*cos(theta2) + l3*cos(theta2 + theta3);
     108.4*sin(0) + l2*sin(theta2) + l3*sin(theta2 + theta3);
     d1];  % d1 es la junta prismática

% Jacobiano simbólico
J = jacobian(P, [d1, theta2, theta3]);

% Sustituir los valores de q en el Jacobiano
J_num = double(subs(J, {d1, theta2, theta3, l2, l3}, {q(1), q(2), q(3), l2, l3}));

% Calcular velocidades cartesianas
xdot = J_num * qdot(:);

end
