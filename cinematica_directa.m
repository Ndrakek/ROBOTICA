function P = cinematica_directa(q)
% =====================================================
% cinematica_directa  Calcula la posición cartesiana
%                     del efector final de un robot PRR.
%
% Entrada:
%   q : vector columna [d1; theta2; theta3]
%
% Salida:
%   P : vector columna [x; y; z]
% =====================================================

% --- Parámetros geométricos ---
l1 = 108.4;
l2 = 82.5;
l3 = 85;

% --- Variables articulares ---
d1 = q(1);
theta2 = q(2);
theta3 = q(3);

% --- Matrices homogéneas ---
A1 = [1 0 0 l1;
      0 1 0 0;
      0 0 1 d1;
      0 0 0 1];

A2 = [cos(theta2) -sin(theta2) 0 l2*cos(theta2);
      sin(theta2)  cos(theta2) 0 l2*sin(theta2);
      0            0           1 0;
      0            0           0 1];

A3 = [cos(theta3) -sin(theta3) 0 l3*cos(theta3);
      sin(theta3)  cos(theta3) 0 l3*sin(theta3);
      0            0           1 0;
      0            0           0 1];

% --- Transformación total ---
T03 = A1 * A2 * A3;

% --- Posición del efector final ---
P = T03(1:3, 4);

end
