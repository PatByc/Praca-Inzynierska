function [q] = InverseKinematics(a, d, T, T_syms, q_actual, force_config, force_config_number)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% Sprawdzenie liczby argumentów wejściowych (nie możesz używać domyślnych wartości parametrów w samej deklaracji funkcji w taki sposób, jak w Pythonie czy C++)
if nargin < 6
    force_config = 0; % Domyślna wartość dla force_config
end
if nargin < 7
    force_config_number = 0; % Domyślna wartość dla force_config_number
end

% Symboliczne zmienne
syms t q1(t) q2(t) q3(t) q4(t) q5(t) q6(t); % Zmienne konfiguracyjne jako funkcje czasu
syms a0 a1 a2 a3 a4 a5;                     % Długości członów
syms d1 d2 d3 d4 d5 d6 dk;                     % Przesunięcia wzdłuż osi Z
syms alpha0 alpha1 alpha2 alpha3 alpha4 alpha5; % Kąty między osiami Z

% Macierze transformacji
T01 = T(:,:,1);
T02 = T(:,:,2);
T03 = T(:,:,3);
T04 = T(:,:,4);
T05 = T(:,:,5);
T06 = T(:,:,6);
T0k = T(:,:,7);

% Macierze transformacji na symbolach
T01_syms = T_syms(:,:,1);
T02_syms = T_syms(:,:,2);
T03_syms = T_syms(:,:,3);
T04_syms = T_syms(:,:,4);
T05_syms = T_syms(:,:,5);
T06_syms = T_syms(:,:,6);
T0k_syms = T_syms(:,:,7);

%--------------------------------------------------------------------------------------------------------------------

Eulers = ToEulers(T0k);

% Przypisanie wartosci obliczonych z Kp do Ko
P_Eulers = Eulers(1:3, 1);
R_Eulers = Eulers(4:6, 1);

% Przestrzen robocza

%{
min_reach = abs(sum(d) - sum(a)); % Minimalny zasięg
max_reach = sum(d) + sum(a); % Maksymalny zasięg
max_radius_xy = max_reach; % Maksymalny promień w płaszczyźnie XY
z_axis_range = abs(sum(d)); % Maksymalny zasięg wzdłuż osi Z

% Wyświetlenie limitów przestrzeni roboczej
disp("Limity przestrzeni roboczej:")
fprintf('Minimalny zasięg: %.2f mm\n', min_reach);
fprintf('Maksymalny zasięg: %.2f mm\n', max_reach);
fprintf('Maksymalny promień w płaszczyźnie XY: %.2f mm\n', max_radius_xy);
fprintf('Zakres osi Z: %.2f mm (W górę od podstawy)\n', z_axis_range);

% Obliczanie odległości od początku układu współrzędnych
distance_from_origin = norm(P_Eulers);
fprintf('Odległość od początku układu współrzędnych: %.2f mm\n', distance_from_origin);
fprintf('\n');

% Sprawdzenie, czy punkt mieści się w przestrzeni roboczej
if distance_from_origin >= min_reach && distance_from_origin <= max_reach
    disp("Punkt docelowy znajduje się w przestrzeni roboczej.");
elseif distance_from_origin > max_reach
    warning("Punkt docelowy znajduje się poza przestrzenią roboczą, punkt jest za daleko!");
elseif distance_from_origin < min_reach
    warning("Punkt docelowy znajduje się poza przestrzenią roboczą, punkt jest za blisko!");
end
fprintf('\n');

% Generowanie wykresu przestrzeni roboczej
N = 1000;        % Liczba losowych punktów

% Generowanie losowych punktów w przestrzeni roboczej
theta = rand(N, 1) * 2 * pi;       % Kąt azymutalny (0 do 2*pi)
phi = acos(2 * rand(N, 1) - 1);    % Kąt biegunowy (0 do pi)
r = min_reach + (max_reach - min_reach) * rand(N, 1); % Promień losowy w zakresie zasięgu

% Przekształcenie do współrzędnych kartezjańskich
x = r .* sin(phi) .* cos(theta);
y = r .* sin(phi) .* sin(theta);
z = r .* cos(phi);

% Tworzenie siatki dla sfer minimalnego i maksymalnego zasięgu
[theta_grid, phi_grid] = meshgrid(linspace(0, 2*pi, 50), linspace(0, pi, 50));
x_min = min_reach * sin(phi_grid) .* cos(theta_grid);
y_min = min_reach * sin(phi_grid) .* sin(theta_grid);
z_min = min_reach * cos(phi_grid);

x_max = max_reach * sin(phi_grid) .* cos(theta_grid);
y_max = max_reach * sin(phi_grid) .* sin(theta_grid);
z_max = max_reach * cos(phi_grid);
%}



%Tworzenie wykresu 3D przestrzeni roboczej
%{
% Tworzenie wykresu 3D przestrzeni roboczej
figure;
hold on;
grid on;

% Rysowanie sfer minimalnego i maksymalnego zasięgu
surf(x_min, y_min, z_min, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'g'); % Zielona sfera (min reach)
surf(x_max, y_max, z_max, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'r'); % Czerwona sfera (max reach)

% Rysowanie losowych punktów
scatter3(x, y, z, 10, 'b', 'filled');

% Oznaczenia osi i tytuł
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Przestrzeń robocza robota 3D');
legend({'Minimal Reach', 'Maximal Reach', 'Random Points'}, 'Location', 'best');

view(3);
axis equal;
hold off;
%}


% Przechowywanie różnych konfiguracji aby obsluzyc nieosobliwosci za pomoca kryterium tolerancji
configs = [];

%------------------------------------------------------------

% Kinematyka przestrzenna - obliczanie q1, q2, q3

% q1

% Wyznaczenie α

% Punkt P (Punkt w ukladzie 4 - q4)
P04 = T04(1:3, 4);

alpha1 = atan2(P04(2), P04(1));

% Wyznaczenie β
sin_beta1 = d(3) / (sqrt(P04(1)^2 + P04(2)^2));
if abs(sin_beta1) > 1
    disp('Punkt poza zakresem - brak rozwiązania dla β!');
end

cos_beta1 = sqrt(1 - sin_beta1^2);

% q1 => q1_A ^ q1_B
beta1_positive = atan2(sin_beta1, cos_beta1);
beta1_negative = atan2(sin_beta1, -cos_beta1);

q1_A = alpha1 - beta1_positive;
q1_B = alpha1 - beta1_negative;

if abs(P04(1)) < 1e-6 && abs(P04(2)) < 1e-6
    % Punkt docelowy leży na osi, kąt q1 jest nieokreślony
    q1_A = 0; % Można przypisać wartość domyślną, np. q1 = 0
    q1_B = 0; % lub inną wartość odpowiednią dla aplikacji
    disp('Punkt docelowy leży na osi robota. Przyjęto q1 = 0.');
end

% Wyświetlenie wyników q1
%{
disp('Rozwiązania dla q1:');
fprintf('q1 A: %.4f rad (%.2f degrees)\n', q1_A,  rad2deg(q1_A));
fprintf('q1 B: %.4f rad (%.2f degrees)\n', q1_B, rad2deg(q1_B));
fprintf('\n');
%}

% q3

% Wzor: P14 = ((T01(q1))^-1 * P04   

% Zmiana P04 na homogeniczne współrzędne, dodanie jedynki jako czwartego elementu
P04 = [P04(1);
       P04(2);
       P04(3);
       1];

% Transformacje dla q1_A i q1_B
T01_q1_A = (double(subs(T01_syms, [q1(t), a0, d1], [q1_A, a(1), d(1)])));
T01_q1_B = (double(subs(T01_syms, [q1(t), a0, d1], [q1_B, a(1), d(1)])));
% Wyznaczenie P14

% Potrzebne P01 oraz R01 z q1_A
P01_q1_A = T01_q1_A(1:3, 4);
R01_q1_A = T01_q1_A(1:3, 1:3);


T01_q1_A_inv = [R01_q1_A', -(R01_q1_A') * P01_q1_A;
                0, 0, 0,                         1;];

% Potrzebne P01 oraz R01 z q1_B
P01_q1_B = T01_q1_B(1:3, 4);
R01_q1_B = T01_q1_B(1:3, 1:3);

T01_q1_B_inv = [R01_q1_B', -(R01_q1_B)' * P01_q1_B;
                0, 0, 0,                         1;];


% Ostateczne wyliczenia P14
P14_q1_A = T01_q1_A_inv * P04;
P14_q1_B = T01_q1_B_inv * P04;

% Wartości geometryczne dla P14
x_A = P14_q1_A(1);
z_A = P14_q1_A(3);

x_B = P14_q1_B(1);
z_B = P14_q1_B(3);


% Standardowy przypadek
cos_beta_q1_A = (x_A^2 + z_A^2 - a(3)^2 - d(4)^2) / (2 * a(3) * d(4));
%cos_beta_q1_A = max(-1, min(1, cos_beta_q1_A)); % Ograniczenie wartości do zakresu [-1, 1]
    
beta_q3_A = acos(cos_beta_q1_A); % beta > 0 dla q1_A
beta_q3_B = -acos(cos_beta_q1_A); % beta < 0 dla q1_A

q3_A = pi/2 + beta_q3_A;
q3_B = pi/2 + beta_q3_B;

% Standardowy przypadek dla q1_B
cos_beta_q1_B = (x_B^2 + z_B^2 - a(3)^2 - d(4)^2) / (2 * a(3) * d(4));
%cos_beta_q1_B = max(-1, min(1, cos_beta_q1_B)); % Ograniczenie wartości do zakresu [-1, 1]

beta_q3_C = acos(cos_beta_q1_B); % beta > 0 dla q1_B
beta_q3_D = -acos(cos_beta_q1_B); % beta > 0 dla q1_B

q3_C = pi/2 + beta_q3_C;
q3_D = pi/2 + beta_q3_D;

% Wyświetlenie wyników q3
%{
disp('Rozwiązania dla q3:');
fprintf('q3 A: %.4f rad (%.2f degrees)\n', q3_A, rad2deg(q3_A));
fprintf('q3 B: %.4f rad (%.2f degrees)\n', q3_B, rad2deg(q3_B));
fprintf('q3 C: %.4f rad (%.2f degrees)\n', q3_C, rad2deg(q3_C));
fprintf('q3 D: %.4f rad (%.2f degrees)\n', q3_D, rad2deg(q3_D));
%}

% q2

% Obliczenie α dla q1_A
alpha2_A = atan2(P14_q1_A(3), P14_q1_A(1));

% Wyznaczenie cos(beta) dla q1_A
r_A = sqrt(P14_q1_A(3)^2 + P14_q1_A(1)^2);
cos_beta_q2_A = ((P14_q1_A(1))^2 + (P14_q1_A(3)^2 + a(3)^2 - d(4)^2)) / (2 * a(3) * r_A);

% Sprawdzenie zakresu cos_beta_q2_A
if abs(cos_beta_q2_A) > 1
    disp('Punkt poza zakresem - brak rozwiązania dla q2_A!');
end


% Obliczenie beta dla q1_A
beta_q2_A = acos(cos_beta_q2_A);
beta_q2_B = acos(cos_beta_q2_A);

% Wyznaczenie q2 dla q1_A - Warunek sprawdzajacy znak beta (roznorodnosc rozwiazan)
if beta_q3_B < 0
    q2_A = - (alpha2_A + beta_q2_A); % Negacja kąta, bo jest on obliczany w złym kierunku
elseif beta_q3_B == 0
    q2_A = - (alpha2_A + beta_q2_A);
    disp("beta_q3_B == 0 -> ");
else
    warning('beta_q3_B > 0 !!!')
end


if beta_q3_A > 0
    q2_B = - (alpha2_A - beta_q2_B); % Negacja kąta, bo jest on obliczany w złym kierunku
elseif beta_q3_A == 0
    q2_B = - (alpha2_A - beta_q2_B); 
    disp("beta_q3_A == 0 -> ");
else
    warning('beta_q3_A < 0 !!!')
end

% Obliczenie α dla q1_B
alpha2_B = atan2(P14_q1_B(3), P14_q1_B(1)); 

% Wyznaczenie cos(beta) dla q1_B
r_B = sqrt(P14_q1_B(3)^2 + P14_q1_B(1)^2);
cos_beta_q2_B = ((P14_q1_B(1))^2 + (P14_q1_B(3)^2 + a(3)^2 - d(4)^2)) / (2 * a(3) * r_B);

% Sprawdzenie zakresu cos_beta_q2_B
if abs(cos_beta_q2_B) > 1
    disp('Punkt poza zakresem - brak rozwiązania dla q2_B!');
end

% Obliczenie beta dla q1_B
beta_q2_C = acos(cos_beta_q2_B);
beta_q2_D = acos(cos_beta_q2_B);

% Wyznaczenie q2 dla q1_B - Warunek sprawdzajacy znak beta (roznorodnosc rozwiazan)
if beta_q3_D < 0
    q2_C = - (alpha2_B + beta_q2_C);
elseif beta_q3_D == 0
    q2_C = - (alpha2_B + beta_q2_C);
    disp("beta_q3_D == 0 -> ");
end

if beta_q3_C > 0
    q2_D = - (alpha2_B - beta_q2_D);
elseif beta_q3_C == 0
    q2_D = - (alpha2_B - beta_q2_D);
    disp("beta_q3_C == 0 -> ");
end

% Wyświetlenie wyników q2
%{
disp('Rozwiązania dla q2:');
fprintf('q2 A: %.4f rad (%.2f degrees)\n', q2_A, rad2deg(q2_A));
fprintf('q2 B: %.4f rad (%.2f degrees)\n', q2_B, rad2deg(q2_B));
fprintf('q2 C: %.4f rad (%.2f degrees)\n', q2_C, rad2deg(q2_C));
fprintf('q2 D: %.4f rad (%.2f degrees)\n', q2_D, rad2deg(q2_D));
fprintf('\n');
%}

% ---------------------------------------------------------------------------

% Kinematyka orientacyjna - obliczanie q4, q5, q6 

% R03 * R3k = R0k => R3k = (RO3)^-1 * R0k => R3k = (R03)^T * R0k
R0k = T06(1:3, 1:3);

% Wyznaczanie 4 roznych brakujacych macierzy R03


syms q1_x q2_x q3_x;

R03 = T03(1:3, 1:3);
%disp('RO3:');
%disp(R03);

R03_x = [cos(q1_x)*cos(q2_x)*cos(q3_x) - cos(q1_x)*sin(q2_x)*sin(q3_x), - cos(q1_x)*cos(q2_x)*sin(q3_x) - cos(q1_x)*cos(q3_x)*sin(q2_x), -sin(q1_x);
         cos(q2_x)*cos(q3_x)*sin(q1_x) - sin(q1_x)*sin(q2_x)*sin(q3_x), - cos(q2_x)*sin(q1_x)*sin(q3_x) - cos(q3_x)*sin(q1_x)*sin(q2_x),  cos(q1_x);
       - cos(q2_x)*sin(q3_x) - cos(q3_x)*sin(q2_x),                       sin(q2_x)*sin(q3_x) - cos(q2_x)*cos(q3_x),                             0];

%disp('R03((q1_x, q2_x, q3_x):')
%disp(R03_x);

R03_A = (double(subs(R03_x, [q1_x, q3_x, q2_x], [q1_A, q3_A, q2_A])));
R03_B = (double(subs(R03_x, [q1_x, q3_x, q2_x], [q1_A, q3_B, q2_B])));
R03_C = (double(subs(R03_x, [q1_x, q3_x, q2_x], [q1_B, q3_C, q2_C])));
R03_D = (double(subs(R03_x, [q1_x, q3_x, q2_x], [q1_B, q3_D, q2_D])));

% Wyznaczenie R3k
R3k_A = (R03_A') * R0k;   % R03_A = RO3
R3k_B = (R03_B') * R0k;
R3k_C = (R03_C') * R0k;
R3k_D = (R03_D') * R0k;


% q5 (8 rozwiazan)

% z R03_A
sq5_A = sqrt((R3k_A(1,3)^2) + (R3k_A(3,3)^2));
cq5_A = -R3k_A(2,3);

q5_A = atan2(sq5_A, cq5_A);
q5_B = atan2(-sq5_A, cq5_A);


% z R03_B
sq5_B = sqrt((R3k_B(1,3)^2) + (R3k_B(3,3)^2));
cq5_B = -R3k_B(2,3);

q5_C = atan2(sq5_B, cq5_B);
q5_D = atan2(-sq5_B, cq5_B);


% z R03_C
sq5_C = sqrt((R3k_C(1,3)^2) + (R3k_C(3,3)^2));
cq5_C = -R3k_C(2,3);

q5_E = atan2(sq5_C, cq5_C);
q5_F = atan2(-sq5_C, cq5_C);


% z R03_D
sq5_D = sqrt((R3k_D(1,3)^2) + (R3k_D(3,3)^2));
cq5_D = -R3k_D(2,3);

q5_G = atan2(sq5_D, cq5_D);
q5_H = atan2(-sq5_D, cq5_D);


% q4 (8 rozwiazan)

% z R03_A
q4_A = atan2((R3k_A(3,3)), (R3k_A(1,3)));
q4_B = atan2(-(R3k_A(3,3)), (-R3k_A(1,3)));

% z R03_B
q4_C = atan2((R3k_B(3,3)), (R3k_B(1,3)));
q4_D = atan2((-R3k_B(3,3)), (-R3k_B(1,3)));

% z R03_C
q4_E = atan2((R3k_C(3,3)), (R3k_C(1,3)));
q4_F = atan2(-(R3k_C(3,3)), (-R3k_C(1,3)));

% z R03_D
q4_G = atan2((R3k_D(3,3)), (R3k_D(1,3)));
q4_H = atan2(-(R3k_D(3,3)), (-R3k_D(1,3)));


% q6 (8 rozwiazan)

% z R03_A
q6_A = atan2((R3k_A(2,2)), (R3k_A(2,1)));
q6_B = atan2((-R3k_A(2,2)), (-R3k_A(2,1)));

% z R03_B
q6_C = atan2((R3k_B(2,2)), (R3k_B(2,1)));
q6_D = atan2((-R3k_B(2,2)), (-R3k_B(2,1)));

% z R03_C
q6_E = atan2((R3k_C(2,2)), (R3k_C(2,1)));
q6_F = atan2((-R3k_C(2,2)), (-R3k_C(2,1)));

% z R03_D
q6_G = atan2((R3k_D(2,2)), (R3k_D(2,1)));
q6_H = atan2((-R3k_D(2,2)), (-R3k_D(2,1)));


% Generowanie 8 konfiguracji 
configs(end+1, :) = [q1_A, q2_A, q3_A, q4_A, q5_A, q6_A, 0];
configs(end+1, :) = [q1_A, q2_A, q3_A, q4_B, q5_B, q6_B, 0];
configs(end+1, :) = [q1_A, q2_B, q3_B, q4_C, q5_C, q6_C, 0];
configs(end+1, :) = [q1_A, q2_B, q3_B, q4_D, q5_D, q6_D, 0];
configs(end+1, :) = [q1_B, q2_C, q3_C, q4_E, q5_E, q6_E, 0];
configs(end+1, :) = [q1_B, q2_C, q3_C, q4_F, q5_F, q6_F, 0];
configs(end+1, :) = [q1_B, q2_D, q3_D, q4_G, q5_G, q6_G, 0];
configs(end+1, :) = [q1_B, q2_D, q3_D, q4_H, q5_H, q6_H, 0];


% Wyswietlanie wszystkich konfiguracji dla porownania z RTB
disp('Dostępne konfiguracje:');

for i = 1:size(configs, 1) % Rozmiar konfiguracji (wierszy)
    fprintf('(%d): ', i); % Wyświetlenie numeru konfiguracji
    fprintf('%.4f ', rad2deg(configs(i, :))); % Wyświetlenie wartości konfiguracji
    fprintf('\n'); % Nowa linia po każdej konfiguracji
end



% Aktualny stan robota (kąty)
current_q = q_actual;
%current_q = [0 0 0 0 0 0];
configs_test = configs;


% Kryterium tolerancji - Wybór konfiguracji z najmniejszą różnicą (poradzenie sobie z mnogoscia rozwiazan wraz z mozliwymi nieosobliwosciami o charakterystycznych nazwach:
min_diff = inf; % Inicjalizacja minimalnej różnicy
for i = 1:size(configs_test, 1)
    diff = norm(configs_test(i, :) - current_q); % Odległość w przestrzeni konfiguracyjnej
    if diff < min_diff
        min_diff = diff;
        preferred_config = configs_test(i, :); % Aktualizacja wybranej konfiguracji
        config_number_test = i; % Numer konfiguracji
    end
end

% Wyświetlenie wybranej konfiguracji
fprintf('Wybrana konfiguracja (najbliższa aktualnemu stanowi) to konfiguracja numer %d:\n', config_number_test);

if force_config == 0
    q = preferred_config;
else
    q = configs(force_config_number, :);
end

% Return
q = q;
end