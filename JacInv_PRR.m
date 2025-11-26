function Jinv = JacInv_PRR(q)
% =====================================================
% JacInv_PRR  Calcula el Jacobiano inverso del
%             manipulador PRR.
%
% Entrada:
%   q : vector columna [d1; theta2; theta3]
%
% Salida:
%   Jinv : matriz Jacobiano inverso (3x3)
% =====================================================

% --- Parámetros geométricos ---
a1 = 108.4;
a2 = 82.5;
a3 = 85;

% --- Variables articulares ---
d1 = q(1);
theta2 = q(2);
theta3 = q(3);

% --- Matrices tipo DH ---
DH = [a1 0 d1 0;
      a2 0 0 theta2;
      a3 0 0 theta3];

% --- Transformaciones homogéneas individuales ---
A = cell(1,3);
for i = 1:3
    a = DH(i,1);
    alpha = DH(i,2);
    d = DH(i,3);
    theta = DH(i,4);

    A{i} = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
            sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
            0           sin(alpha)             cos(alpha)             d;
            0           0                      0                      1];
end

% --- Transformaciones acumuladas ---
T01 = A{1};
T02 = T01 * A{2};
T03 = T02 * A{3};

% --- Posiciones de cada marco ---
p1 = T01(1:3,4);
p2 = T02(1:3,4);
pe = T03(1:3,4);

% --- Ejes z ---
z0 = [0;0;1];
z1 = T01(1:3,3);
z2 = T02(1:3,3);

% --- Columnas del Jacobiano lineal ---
Jv1 = z0;                       
Jv2 = cross(z1, (pe - p1));
Jv3 = cross(z2, (pe - p2));

% --- Jacobiano total ---
Jv = [Jv1 Jv2 Jv3];

% --- Jacobiano inverso ---
Jinv = pinv(Jv);   % (3x3)

end
