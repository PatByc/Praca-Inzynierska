function [Eulers] = ToEulers(T0k)

% Definiowanie symbolicznych zmiennych
alfa = sym('alfa');
beta = sym('beta');
gama = sym('gama');

Rz_alfa = [cos(alfa), -sin(alfa), 0;
           sin(alfa), cos(alfa),  0;
           0,         0,          1;];

Ry_beta = [cos(beta), 0, sin(beta);
           0,         1,         0;
           -sin(beta), 0, cos(beta);];

Rz_gama = [cos(gama), -sin(gama), 0;
           sin(gama), cos(gama),  0;
           0,         0,          1;];

Rzyz = Rz_alfa * Ry_beta * Rz_gama;

%{
disp('Macierz Rzyz do obliczenia katow Eulera po ZYZ:')
disp(Rzyz);
%}

P0k = T0k(1:3, 4);
R0k = T0k(1:3, 1:3);

pitch = atan2(sqrt(R0k(1,3)^2 + R0k(2,3)^2), R0k(3,3));

roll = atan2(R0k(2,3), R0k(1,3));



yaw = atan2(R0k(3,2), -R0k(3,1));

% Polozenie w reprezentacji Eulera (pozycja + orientacja)
Eulers = [P0k(1,1);
          P0k(2,1);
          P0k(3,1);
          roll;
          pitch;
          yaw;];

% Wyświetlanie wyników w w reprezentacji Eulera w stopniach (z dokładnością do 2 miejsc po przecinku)

%{
fprintf('\n');
disp('Polozenie w reprezentacji Eulera (stopnie)')
fprintf('x: %.2f \n', Eulers(1));
fprintf('y: %.2f \n', Eulers(2));
fprintf('z: %.2f \n', Eulers(3));

fprintf('Roll (φ): %.2f deg\n', rad2deg(Eulers(4)));
fprintf('Pitch (θ): %.2f deg\n', rad2deg(Eulers(5)));
fprintf('Yaw (ψ): %.2f deg\n', rad2deg(Eulers(6)));
%}


end