function [x, y, z, plots, quivers] = AnimateRobot(T)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Macierze transformacji
T01 = T(:,:,1);
T02 = T(:,:,2);
T03 = T(:,:,3);
T04 = T(:,:,4);
T05 = T(:,:,5);
T06 = T(:,:,6);
T0k = T(:,:,7);

P01 = T01(1:3, 4);
P02 = T02(1:3, 4);
P03 = T03(1:3, 4);
P04 = T04(1:3, 4);
P05 = T05(1:3, 4);
P06 = T06(1:3, 4);
P0k = T0k(1:3, 4);


% Współrzędne wektorów
P01_start = [0, 0, 0]; % Punkt zaczepienia P01
P01_end = P01';   % Punkt końcowy wektora P01

P02_start = P01_end;      
P02_end = P02';

P03_start = P02_end;      
P03_end = P03';

P04_start = P03_end;      
P04_end = P04';

P05_start = P04_end;      
P05_end = P05';

P06_start = P05_end;      
P06_end = P06';

P0k_start = P06_end;      
P0k_end = P0k';

x = [P01_start(1), P02_start(1), P03_start(1), P04_start(1), P05_start(1), P06_start(1), P0k_start(1) , P0k_end(1)];
y = [P01_start(2), P02_start(2), P03_start(2), P04_start(2), P05_start(2), P06_start(2), P0k_start(2) , P0k_end(2)];
z = [P01_start(3), P02_start(3), P03_start(3), P04_start(3), P05_start(3), P06_start(3), P0k_start(3) , P0k_end(3)];

% Rysowanie wektorów
plots = zeros(7, 6);
plots = [P01_start(1), P01_end(1), P01_start(2), P01_end(2), P01_start(3), P01_end(3);
         P02_start(1), P02_end(1), P02_start(2), P02_end(2), P02_start(3), P02_end(3);
         P03_start(1), P03_end(1), P03_start(2), P03_end(2), P03_start(3), P03_end(3);
         P04_start(1), P04_end(1), P04_start(2), P04_end(2), P04_start(3), P04_end(3);
         P05_start(1), P05_end(1), P05_start(2), P05_end(2), P05_start(3), P05_end(3);
         P06_start(1), P06_end(1), P06_start(2), P06_end(2), P06_start(3), P06_end(3);
         P0k_start(1), P0k_end(1), P0k_start(2), P0k_end(2), P0k_start(3), P0k_end(3);];

% Rysowanie osi w punkcie TCP
T0k = T(:,:,7); % Końcowa macierz transformacji
TCP_position = T0k(1:3, 4); % Pozycja TCP
R = T0k(1:3, 1:3); % Rotacja końcowego układu współrzędnych

% Wektory osi układu lokalnego
x_axis = R(:, 1); % Oś X
y_axis = R(:, 2); % Oś Y
z_axis = R(:, 3); % Oś Z

% Skalowanie długości osi
scale = 1; % Długość osi (do dopasowania)
x_axis = scale * x_axis;
y_axis = scale * y_axis;
z_axis = scale * z_axis;

% Rysowanie osi w punkcie TCP
quivers = zeros(1, 7, 3);
quivers = [TCP_position(1), TCP_position(2), TCP_position(3), x_axis(1), x_axis(2), x_axis(3);
           TCP_position(1), TCP_position(2), TCP_position(3), y_axis(1), y_axis(2), y_axis(3);
           TCP_position(1), TCP_position(2), TCP_position(3), z_axis(1), z_axis(2), z_axis(3);];

end