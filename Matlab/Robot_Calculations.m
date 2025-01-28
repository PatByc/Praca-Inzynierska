clear all;
close all;

disp("----------------------- RUN PROGRAM -----------------------");

% Wektory parametrow ZDH
% katy alpha = [alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, 0];
alpha = [0, -pi/2, 0, pi/2, -pi/2, pi/2, 0];

% zmienne konfiguracyjne q = [q1(t), q2(t), q3(t), q4(t), q5(t), q6(t), 0];
%q = [q1(t), q2(t), q3(t), q4(t), q5(t), q6(t)];

% PARAMETRY PO KONSULTACJACH

% dlugosci a = [a0, a1, a2, a3, a4, a5, 0];
a = [0, 0, 105, 0, 0, 0, 0];

% przesuniecia d = [d1, d2, d3, d4, d5, d6, dk];
d = [100, 0, 0, 90, 0, 40, 80];



figure_x = 250;
figure_y = 250;

% Tabela ZDH
ZDH_Parameters(alpha, a, d);

% Duza macierz T, do przesylania wszystkich macierzy transformacji
T = zeros(4, 4, 7);

% Poczatkowa konfiguracja
q_actual = [0, 0, 0, 0, 0, 0, 0];
[Eulers_P_actual, T_P_actual, T_syms_P_actual] = ForwardKinematics(alpha, a, q_actual, d);
DrawRobot(T_P_actual, "Point for configs [0, 0, 0, 0, 0, 0, 0]", figure_x*0, figure_y*0, figure_x, figure_y);

% Odkomentowac, aby wyswietlic wszystkie konfiguracje

%{

[Eulers, T, T_syms] = ForwardKinematics(alpha, a, q_actual, d);

% IK dla 8 konfiguracji
q_out_test_actual = [0, 0, 0, 0, 0, 0, 0];
q_out_test = InverseKinematics(a, d, T, T_syms, q_out_test_actual, 1, 1);

disp('testing for config 1');
[Eulers1, T1, T1_syms] = ForwardKinematics(alpha, a, q_out_test, d);

q_out_test = InverseKinematics(a, d, T, T_syms, q_out_test_actual, 1, 2);
disp('testing for config 2');
[Eulers2, T2, T2_syms] = ForwardKinematics(alpha, a, q_out_test, d);

q_out_test = InverseKinematics(a, d, T, T_syms, q_out_test_actual, 1, 3);
disp('testing for config 3');
[Eulers3, T3, T3_syms] = ForwardKinematics(alpha, a, q_out_test, d);

q_out_test = InverseKinematics(a, d, T, T_syms, q_out_test_actual, 1, 4);
disp('testing for config 4');
[Eulers4, T4, T4_syms] = ForwardKinematics(alpha, a, q_out_test, d);

q_out_test = InverseKinematics(a, d, T, T_syms, q_out_test_actual, 1, 5);
disp('testing for config 5');
[Eulers5, T5, T5_syms] = ForwardKinematics(alpha, a, q_out_test, d);

q_out_test = InverseKinematics(a, d, T, T_syms, q_out_test_actual, 1, 6);
disp('testing for config 6');
[Eulers6, T6, T6_syms] = ForwardKinematics(alpha, a, q_out_test, d);

q_out_test = InverseKinematics(a, d, T, T_syms, q_out_test_actual, 1, 7);
disp('testing for config 7');
[Eulers7, T7, T7_syms] = ForwardKinematics(alpha, a, q_out_test, d);

q_out_test = InverseKinematics(a, d, T, T_syms, q_out_test_actual, 1, 8);
disp('testing for config 8');
[Eulers8, T8, T8_syms] = ForwardKinematics(alpha, a, q_out_test, d);

figure_x = 250;
figure_y = 250;

% DrawRobot(T5, "config 5", figure_pos_x(5,1), figure_pos_y(5,1), figure_x, figure_y)
DrawRobot(T1, "config 1", figure_x*0, figure_y*0, figure_x, figure_y);
DrawRobot(T2, "config 2", figure_x*1, figure_y*0, figure_x, figure_y);
DrawRobot(T3, "config 3", figure_x*2, figure_y*0, figure_x, figure_y);
DrawRobot(T4, "config 4", figure_x*3, figure_y*0, figure_x, figure_y);
DrawRobot(T5, "config 5", figure_x*0, figure_y*1, figure_x, figure_y);
DrawRobot(T6, "config 6", figure_x*1, figure_y*1, figure_x, figure_y);
DrawRobot(T7, "config 7", figure_x*2, figure_y*1, figure_x, figure_y);
DrawRobot(T8, "config 8", figure_x*3, figure_y*1, figure_x, figure_y);

%}




% Porownanie mojej FK a RTB FK

q_actual = [0, 0, 0, 0, 0, 0, 0];

                 % [alpha_i_1 / a_i_1 / theta_i = q / d_i]
L(1) = RevoluteMDH('alpha', alpha(1), 'a', a(1), 'd', d(1));
L(2) = RevoluteMDH('alpha', alpha(2), 'a', a(2), 'd', d(2));
L(3) = RevoluteMDH('alpha', alpha(3), 'a', a(3), 'd', d(3));
L(4) = RevoluteMDH('alpha', alpha(4), 'a', a(4), 'd', d(4));
L(5) = RevoluteMDH('alpha', alpha(5), 'a', a(5), 'd', d(5));
L(6) = RevoluteMDH('alpha', alpha(6), 'a', a(6), 'd', d(6));
L(7) = RevoluteMDH('alpha', alpha(7), 'a', a(7), 'd', d(7)); % DOROBIENIE KONCOWKI NA KONCU

Robot = SerialLink(L);
Robot.name = '6DOF Robot';

figure;
% Wyświetlenie robota w tej konfiguracji

TCP = Robot.fkine(q_actual);
Robot.plot(q_actual); % Rysowanie robota

% Wyświetlenie frame robota (world frame) z boku

hold on;
trplot(eye(4), 'frame', 'W', 'length', 2, 'color', 'k'); % World frame w (0,0,0)
hold off;



% Oznaczenia osi i tytuł
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Robot with World Frame');

% Robot.teach();


% ---------------------------





% home
q_P1 = [0.5027, -1.9129, 2.8676, 0.0000, 0.6248, 0.0000, 0.0];
[Eulers_P1, T_P1, T_syms_P1] = ForwardKinematics(alpha, a, q_P1, d);
DrawRobot(T_P1, "P1", figure_x*0, figure_y*0, figure_x, figure_y);

% line
q_P12 = [0.5027, 0.0873, 2.1258, 2.0106, 1.0018, -0.2513, 0.0];
[Eulers_P12, T_P12, T_syms_P10] = ForwardKinematics(alpha, a, q_P12, d);
DrawRobot(T_P12, "P12", figure_x*0, figure_y*0, figure_x, figure_y);

% P_R0 (punkt przed regalem gora)
q_P_R0 = [1.4, -1.9129, 2.8676, 0.0000, 0.6248, 0.0000, 0.0];
[Eulers_P_R0, T_P_R0, T_syms_P_R0] = ForwardKinematics(alpha, a, q_P_R0, d);
DrawRobot(T_P_R0, "PR0", figure_x*0, figure_y*0, figure_x, figure_y);

% P_R1 (regal gora)
q_P_R1 = [1.4   -0.5411    2.0944 0 0 0 0 ];
[Eulers_P_R1, T_P_R1, T_syms_P_R1] = ForwardKinematics(alpha, a, q_P_R1, d);
DrawRobot(T_P_R1, "PR1", figure_x*0, figure_y*0, figure_x, figure_y);




%{


% uzywam metody Spherical Linear Interpolation (SLERP) do interpolacji macierzy T06 pomiedzy dwoma punktami
T_interp = SLERP_interpolation(T_P1, T_P10, 5);

DrawRobot(T_P1, "P1", figure_x*0, figure_y*0, figure_x, figure_y);

q_P2 = InverseKinematics(a, d, T_interp(:,:,:,1), T_syms_P1, q_P1);
DrawRobot(T_interp(:,:,:,1), "P2", figure_x*0, figure_y*0, figure_x, figure_y);

q_P3 = InverseKinematics(a, d, T_interp(:,:,:,2), T_syms_P1, q_P2);
DrawRobot(T_interp(:,:,:,2), "P3", figure_x*0, figure_y*0, figure_x, figure_y);

q_P4 = InverseKinematics(a, d, T_interp(:,:,:,3), T_syms_P1, q_P3);
DrawRobot(T_interp(:,:,:,3), "P4", figure_x*0, figure_y*0, figure_x, figure_y);

q_P5 = InverseKinematics(a, d, T_interp(:,:,:,4), T_syms_P1, q_P4);
DrawRobot(T_interp(:,:,:,4), "P5", figure_x*0, figure_y*0, figure_x, figure_y);

q_P6 = InverseKinematics(a, d, T_interp(:,:,:,5), T_syms_P1, q_P5);
DrawRobot(T_interp(:,:,:,5), "P6", figure_x*0, figure_y*0, figure_x, figure_y);

DrawRobot(T_P10, "P10", figure_x*0, figure_y*0, figure_x, figure_y);

%}


% ---------------------------------------------------------------------------------------------------------------
% Path

% Reczne zaimplementowanie punktow

%{

% Path home -> line

% uzywam metody Spherical Linear Interpolation (SLERP) do interpolacji macierzy T06 pomiedzy dwoma punktami
T_interp_1 = SLERP_interpolation(T_P1, T_P12, 10);
q_P2 = InverseKinematics(a, d, T_interp_1(:,:,:,1), T_syms_P1, q_P1);
q_P3 = InverseKinematics(a, d, T_interp_1(:,:,:,2), T_syms_P1, q_P2);
q_P4 = InverseKinematics(a, d, T_interp_1(:,:,:,3), T_syms_P1, q_P3);
q_P5 = InverseKinematics(a, d, T_interp_1(:,:,:,4), T_syms_P1, q_P4);
q_P6 = InverseKinematics(a, d, T_interp_1(:,:,:,5), T_syms_P1, q_P5);
q_P7 = InverseKinematics(a, d, T_interp_1(:,:,:,6), T_syms_P1, q_P6);
q_P8 = InverseKinematics(a, d, T_interp_1(:,:,:,7), T_syms_P1, q_P7);
q_P9 = InverseKinematics(a, d, T_interp_1(:,:,:,8), T_syms_P1, q_P8);
q_P10 = InverseKinematics(a, d, T_interp_1(:,:,:,9), T_syms_P1, q_P9);
q_P11 = InverseKinematics(a, d, T_interp_1(:,:,:,10), T_syms_P1, q_P10);



% Path line -> home
T_interp_2 = SLERP_interpolation(T_P12, T_P1, 10);
q_P12 = InverseKinematics(a, d, T_interp_2(:,:,:,1), T_syms_P1, q_P10);
q_P13 = InverseKinematics(a, d, T_interp_2(:,:,:,2), T_syms_P1, q_P11);
q_P14 = InverseKinematics(a, d, T_interp_2(:,:,:,3), T_syms_P1, q_P12);
q_P15 = InverseKinematics(a, d, T_interp_2(:,:,:,4), T_syms_P1, q_P13);
q_P16 = InverseKinematics(a, d, T_interp_2(:,:,:,5), T_syms_P1, q_P14);
q_P17 = InverseKinematics(a, d, T_interp_2(:,:,:,6), T_syms_P1, q_P15);
q_P18 = InverseKinematics(a, d, T_interp_2(:,:,:,7), T_syms_P1, q_P16);
q_P19 = InverseKinematics(a, d, T_interp_2(:,:,:,8), T_syms_P1, q_P17);
q_P20 = InverseKinematics(a, d, T_interp_2(:,:,:,9), T_syms_P1, q_P18);
q_P21 = InverseKinematics(a, d, T_interp_2(:,:,:,10), T_syms_P1, q_P19);


% Path home -> P0
T_interp_3 = SLERP_interpolation(T_P1, T_P_R0, 10);
q_P21 = InverseKinematics(a, d, T_interp_3(:,:,:,1), T_syms_P1, q_P1);
q_P22 = InverseKinematics(a, d, T_interp_3(:,:,:,2), T_syms_P1, q_P21);
q_P23 = InverseKinematics(a, d, T_interp_3(:,:,:,3), T_syms_P1, q_P22);
q_P24 = InverseKinematics(a, d, T_interp_3(:,:,:,4), T_syms_P1, q_P23);
q_P25 = InverseKinematics(a, d, T_interp_3(:,:,:,5), T_syms_P1, q_P24);
q_P26 = InverseKinematics(a, d, T_interp_3(:,:,:,6), T_syms_P1, q_P25);
q_P27 = InverseKinematics(a, d, T_interp_3(:,:,:,7), T_syms_P1, q_P26);
q_P28 = InverseKinematics(a, d, T_interp_3(:,:,:,8), T_syms_P1, q_P27);
q_P29 = InverseKinematics(a, d, T_interp_3(:,:,:,9), T_syms_P1, q_P28);
q_P30 = InverseKinematics(a, d, T_interp_3(:,:,:,10), T_syms_P1, q_P29);


% Path P0 -> P1
T_interp_4 = SLERP_interpolation(T_P_R0, T_P_R1, 10);
q_P31 = InverseKinematics(a, d, T_interp_4(:,:,:,1), T_syms_P1, q_P_R0);
q_P32 = InverseKinematics(a, d, T_interp_4(:,:,:,2), T_syms_P1, q_P31);
q_P33 = InverseKinematics(a, d, T_interp_4(:,:,:,3), T_syms_P1, q_P32);
q_P34 = InverseKinematics(a, d, T_interp_4(:,:,:,4), T_syms_P1, q_P33);
q_P35 = InverseKinematics(a, d, T_interp_4(:,:,:,5), T_syms_P1, q_P34);
q_P36 = InverseKinematics(a, d, T_interp_4(:,:,:,6), T_syms_P1, q_P35);
q_P37 = InverseKinematics(a, d, T_interp_4(:,:,:,7), T_syms_P1, q_P36);
q_P38 = InverseKinematics(a, d, T_interp_4(:,:,:,8), T_syms_P1, q_P37);
q_P39 = InverseKinematics(a, d, T_interp_4(:,:,:,9), T_syms_P1, q_P38);
q_P40 = InverseKinematics(a, d, T_interp_4(:,:,:,10), T_syms_P1, q_P39);

%}

% -------------------------------------------------------------------------------------------------------------------
% Iteracyjne zaimplementowanie punktow


% Liczba punktów interpolacji pomiedzy zdefiniowanymi punktami
num_points = 40;

% Tablica przechowująca wyniki kątów
q_results = zeros(num_points * 4 + 1, 7); % Każda ścieżka zawiera 10 punktów interpolacji

% Path home -> line
T_interp_1 = SLERP_interpolation(T_P1, T_P12, num_points);
q_results(1, :) = q_P1; % Pierwszy punkt
for i = 1:num_points
    q_results(i + 1, :) = InverseKinematics(a, d, T_interp_1(:,:,:,i), T_syms_P1, q_results(i, :));
end

% Path line -> home
T_interp_2 = SLERP_interpolation(T_P12, T_P1, num_points);
start_index = num_points + 1; % Kontynuacja indeksu
for i = 1:num_points
    q_results(start_index + i, :) = InverseKinematics(a, d, T_interp_2(:,:,:,i), T_syms_P1, q_results(start_index + i - 1, :));
end

% Path home -> P0
T_interp_3 = SLERP_interpolation(T_P1, T_P_R0, num_points);
start_index = 2 * num_points + 1; % Kontynuacja indeksu
for i = 1:num_points
    q_results(start_index + i, :) = InverseKinematics(a, d, T_interp_3(:,:,:,i), T_syms_P1, q_results(start_index + i - 1, :));
end

% Path P0 -> P1
T_interp_4 = SLERP_interpolation(T_P_R0, T_P_R1, num_points);
start_index = 3 * num_points + 1; % Kontynuacja indeksu
for i = 1:num_points
    q_results(start_index + i, :) = InverseKinematics(a, d, T_interp_4(:,:,:,i), T_syms_P1, q_results(start_index + i - 1, :));
end


% ---------------------------------------------------------------------------------------------------------
% Tworzenie animacji

% Zlozenie wszystkich macierzy T_interp w jedna duza
T_interp = cat(4, T_interp_1, T_interp_2, T_interp_3, T_interp_4);

% Inicjalizacja wektorów/macierzy
points_number = 10*4;

x_vec = zeros(1, 8, points_number); % Usunięcie 1 w pierwszym wymiarze
y_vec = zeros(1, 8, points_number);
z_vec = zeros(1, 8, points_number);
plots_vec = zeros(7, 6, points_number); % Tworzenie tablicy komórek
quivers_vec = zeros(3, 6, points_number); 

% Przypisywanie wyników do macierzy
[x, y, z, plots, quivers] = AnimateRobot(T_P1);
x_vec(:, :, 1) = x;
y_vec(:, :, 1) = y;
z_vec(:, :, 1) = z;
plots_vec(:, :, 1) = plots; % Przechowywanie wyników
quivers_vec(:, :, 1) = quivers;



if points_number > 2
    for i = 2:points_number
        [x, y, z, plots, quivers] = AnimateRobot(T_interp(:, :, :, (i-1)));
        x_vec(:, :, i) = x;
        y_vec(:, :, i) = y;
        z_vec(:, :, i) = z;
        plots_vec(:, :, i) = plots;
        quivers_vec(:, :, i) = quivers;
    end
end



[x, y, z, plots, quivers] = AnimateRobot(T_P_R1);
x_vec(:, :, end) = x;
y_vec(:, :, end) = y;
z_vec(:, :, end) = z;
plots_vec(:, :, end) = plots;
quivers_vec(:, :, end)  = quivers;


[x, y, z, plots, quivers] = AnimateRobot(T_P_R1);
x_vec(:, :, end) = x;
y_vec(:, :, end) = y;
z_vec(:, :, end) = z;
plots_vec(:, :, end) = plots;
quivers_vec(:, :, end)  = quivers;


% Tworzenie obiektu wideo
video = VideoWriter('animacja.avi');
video.FrameRate = 5; % Liczba klatek na sekundę
open(video);

% Tworzenie figury i inicjalizacja wykresu
figure;
hold on;
grid on;
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
axis([-100 300 -100 500 0 500]);
view(3); % Ustaw widok w 3D

% Prealokacja uchwytów
title_handle = title('Animacja Trajektorii - P'); % Uchwyt do tytułu
h_scatter = scatter3(x_vec(:, :, 1), y_vec(:, :, 1), z_vec(:, :, 1), 'filled');
h_line = plot3(x_vec(:, :, 1), y_vec(:, :, 1), z_vec(:, :, 1), '-o');

for i = 1:7
    h_plot(i) = plot3([0, 1], [0, 1], [0, 1], 'b', 'LineWidth', 2); % Uchwyt do każdego wektora
end

for i = 1:7
    h_quiver(i) = quiver3(0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 1); % Uchwyt do każdego wektora
end

% Inicjalizacja uchwytów dla opisów osi
h_text_axes = gobjects(3, 1);
h_text_axes(1) = text(0, 0, 0, 'X', 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
h_text_axes(2) = text(0, 0, 0, 'Y', 'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');
h_text_axes(3) = text(0, 0, 0, 'Z', 'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold');

% Inicjalizacja uchwytów dla numerów punktów
h_text_points = gobjects(8, 1);
for i = 1:8
    h_text_points(i) = text(0, 0, 0, sprintf('%d', i-1), 'FontSize', 10, 'FontWeight', 'bold', ...
                            'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
end

% Inicjalizacja uchwytu dla tekstu TCP
h_text_tcp = text(0, 0, 0, '', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k', ...
                  'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

% Pętla animacji
for k = 1:points_number
    set(title_handle, 'String', ['Animacja Trajektorii - P', num2str(k)]);

    % Aktualizacja scatter3 i plot3
    set(h_scatter, 'XData', x_vec(:, :, k), 'YData', y_vec(:, :, k), 'ZData', z_vec(:, :, k));
    set(h_line, 'XData', x_vec(:, :, k), 'YData', y_vec(:, :, k), 'ZData', z_vec(:, :, k));

    % Aktualizacja uchwytu wektorów
    for i = 1:7
        set(h_plot(i), ...
            'XData', [plots_vec(i, 1, k), plots_vec(i, 2, k)], ...
            'YData', [plots_vec(i, 3, k), plots_vec(i, 4, k)], ...
            'ZData', [plots_vec(i, 5, k), plots_vec(i, 6, k)]);
    end

    % Aktualizacja uchwytu do TCP
    for i = 1:3
        if i == 1
            color = 'r';
        elseif i == 2
            color = 'g';
        elseif i == 3
            color = 'b';
        end

        set(h_quiver(i), ...
            'XData', quivers_vec(i, 1, k), ...
            'YData', quivers_vec(i, 2, k), ...
            'ZData', quivers_vec(i, 3, k), ...
            'UData', quivers_vec(i, 4, k), ...
            'VData', quivers_vec(i, 5, k), ...
            'WData', quivers_vec(i, 6, k), ...
            'Color', color);
    end

    % Aktualizacja opisów osi
    set(h_text_axes(1), ...
        'Position', [quivers_vec(1, 1, k) + quivers_vec(1, 4, k), ...
                     quivers_vec(1, 2, k) + quivers_vec(1, 5, k), ...
                     quivers_vec(1, 3, k) + quivers_vec(1, 6, k)]);
    set(h_text_axes(2), ...
        'Position', [quivers_vec(2, 1, k) + quivers_vec(2, 4, k), ...
                     quivers_vec(2, 2, k) + quivers_vec(2, 5, k), ...
                     quivers_vec(2, 3, k) + quivers_vec(2, 6, k)]);
    set(h_text_axes(3), ...
        'Position', [quivers_vec(3, 1, k) + quivers_vec(3, 4, k), ...
                     quivers_vec(3, 2, k) + quivers_vec(3, 5, k), ...
                     quivers_vec(3, 3, k) + quivers_vec(3, 6, k)]);

    % Aktualizacja numerów punktów
    for i = 1:8
        set(h_text_points(i), ...
            'Position', [x_vec(1, i, k), y_vec(1, i, k), z_vec(1, i, k)]);
    end

    % Aktualizacja tekstu TCP
    TCP_text = sprintf('TCP [%.2f, %.2f, %.2f]', x_vec(1, end, k), y_vec(1, end, k), z_vec(1, end, k));
    set(h_text_tcp, ...
        'Position', [x_vec(1, end, k), y_vec(1, end, k), z_vec(1, end, k)], ...
        'String', TCP_text);

    % Zapisanie klatki do wideo
    frame = getframe(gcf);
    writeVideo(video, frame);

    % Pauza (opcjonalna)
    pause(0.5);
end

% Zamknięcie obiektu wideo
close(video);

disp('Animacja zapisana jako "animacja.avi".');


