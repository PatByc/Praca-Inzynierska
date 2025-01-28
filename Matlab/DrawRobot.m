function [] = DrawRobot(T, name, figure_pos_x, figure_pos_y, figure_x, figure_y)
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

% TO
P0k_start = P06_end;      
P0k_end = P0k';

x = [P01_start(1), P02_start(1), P03_start(1), P04_start(1), P05_start(1), P06_start(1), P0k_start(1) , P0k_end(1)];
y = [P01_start(2), P02_start(2), P03_start(2), P04_start(2), P05_start(2), P06_start(2), P0k_start(2) , P0k_end(2)];
z = [P01_start(3), P02_start(3), P03_start(3), P04_start(3), P05_start(3), P06_start(3), P0k_start(3) , P0k_end(3)];

% Wykres 3D
figure('Position', [figure_pos_x, figure_pos_y, figure_x, figure_y+30]); % Pozycja i rozmiar figury
scatter3(x, y, z, 'filled'); % Punkty zaznaczone jako kropki wypełnione
hold on; % Utrzymanie wykresu, aby dodać inne elementy
plot3(x, y, z, '-o'); % Połączenie punktów liniami



% Oznaczenie punktów od 0 do 7
for i = 1:length(x)
    point_label = sprintf('%d', i-1); % Numer punktu
    text(x(i), y(i), z(i), point_label, 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
end

% Rysowanie wektorów
plot3([P01_start(1), P01_end(1)], [P01_start(2), P01_end(2)], [P01_start(3), P01_end(3)], 'b', 'LineWidth', 2);
plot3([P02_start(1), P02_end(1)], [P02_start(2), P02_end(2)], [P02_start(3), P02_end(3)], 'b', 'LineWidth', 2);
plot3([P03_start(1), P03_end(1)], [P03_start(2), P03_end(2)], [P03_start(3), P03_end(3)], 'b', 'LineWidth', 2);
plot3([P04_start(1), P04_end(1)], [P04_start(2), P04_end(2)], [P04_start(3), P04_end(3)], 'b', 'LineWidth', 2);
plot3([P05_start(1), P05_end(1)], [P05_start(2), P05_end(2)], [P05_start(3), P05_end(3)], 'b', 'LineWidth', 2);
plot3([P06_start(1), P06_end(1)], [P06_start(2), P06_end(2)], [P06_start(3), P06_end(3)], 'b', 'LineWidth', 2);
plot3([P0k_start(1), P0k_end(1)], [P0k_start(2), P0k_end(2)], [P0k_start(3), P0k_end(3)], 'b', 'LineWidth', 2);


% Oznaczenie ostatniego punktu
TCP_text = sprintf('TCP [%.2f, %.2f, %.2f]\n', x(end), y(end), z(end)); % Tekst z wartościami Y i Z
text(x(end), y(end), z(end), TCP_text, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

% Rysowanie osi w punkcie TCP
T0k = T(:,:,7); % Końcowa macierz transformacji
TCP_position = T0k(1:3, 4); % Pozycja TCP
R = T0k(1:3, 1:3); % Rotacja końcowego układu współrzędnych

% Wektory osi układu lokalnego
x_axis = R(:, 1); % Oś X
y_axis = R(:, 2); % Oś Y
z_axis = R(:, 3); % Oś Z

% Skalowanie długości osi
scale = 100; % Długość osi (do dopasowania)
x_axis = scale * x_axis;
y_axis = scale * y_axis;
z_axis = scale * z_axis;

% Rysowanie osi w punkcie TCP
quiver3(TCP_position(1), TCP_position(2), TCP_position(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 1); % Oś X
quiver3(TCP_position(1), TCP_position(2), TCP_position(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 1); % Oś Y
quiver3(TCP_position(1), TCP_position(2), TCP_position(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 1); % Oś Z

% Dodanie opisu osi
text(TCP_position(1) + x_axis(1), TCP_position(2) + x_axis(2), TCP_position(3) + x_axis(3), 'X', 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
text(TCP_position(1) + y_axis(1), TCP_position(2) + y_axis(2), TCP_position(3) + y_axis(3), 'Y', 'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');
text(TCP_position(1) + z_axis(1), TCP_position(2) + z_axis(2), TCP_position(3) + z_axis(3), 'Z', 'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold');


% Siatka i etykiety
grid on;
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title(name);

% Tworzenie tekstu do legendy
tcp_text = sprintf('TCP [%.2f, %.2f, %.2f]', x(end), y(end), z(end));
legend({tcp_text}, 'Location', 'best');


hold off;

% Return



end