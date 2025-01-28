function [Eulers, T, T_syms] = ForwardKinematics(alpha, a, q, d)

% Symboliczne zmienne
syms t q1(t) q2(t) q3(t) q4(t) q5(t) q6(t); % Zmienne konfiguracyjne jako funkcje czasu
syms a0 a1 a2 a3 a4 a5;                     % Długości członów
syms d1 d2 d3 d4 d5 d6 dk;                     % Przesunięcia wzdłuż osi Z
syms alpha0 alpha1 alpha2 alpha3 alpha4 alpha5; % Kąty między osiami Z


% katy alpha = [alpha0, alpha1, alpha2, alpha3, alpha4, alpha5]; zapisane symbolicznie w radianach
alpha_syms = [sym(alpha(1)), sym(alpha(2)), sym(alpha(3)), sym(alpha(4)), sym(alpha(5)), sym(alpha(6)), 0];

% zmienne konfiguracyjne q = [q1(t), q2(t), q3(t), q4(t), q5(t), q6(t), 0];
q_syms = [q1(t), q2(t), q3(t), q4(t), q5(t), q6(t), 0];

% symboliczna macierz a
a_syms = [a0, a1, a2, a3, a4, a5, 0];

d_syms = [d1, d2, d3, d4, d5, d6, dk];


% Pętla sprawdzająca i modyfikująca wartości a
for i = 1:length(a)
    if a(i) == 0
        a_syms(i) = 0; % Podstawienie 0 w macierzy a_syms
    end
end

% Pętla sprawdzająca i modyfikująca wartości d
for i = 1:length(a)
    if d(i) == 0
        d_syms(i) = 0; % Podstawienie 0 w macierzy d_syms
    end
end


% Tworzenie 7 macierzy transformacji (T01, T12, T23, T34, T45, T56, T6k) o rozmiarze 4x4 wypełnionej zerami (symboliczne)
T_syms = sym(zeros(4, 4, 7));

% Liczenie macierzy transformacji (symbolicznie)
for i = 1:7

    T_syms(:, :, i) = simplify([ ...
        cos(q_syms(i)),               -sin(q_syms(i)),                0,                a_syms(i);
        sin(q_syms(i))*cos(alpha_syms(i)),  cos(q_syms(i))*cos(alpha_syms(i)), -sin(alpha_syms(i)),   -d_syms(i)*sin(alpha_syms(i));
        sin(q_syms(i))*sin(alpha_syms(i)),  cos(q_syms(i))*sin(alpha_syms(i)),  cos(alpha_syms(i)),    d_syms(i)*cos(alpha_syms(i));
        0,                        0,                        0,                1]);
end


%{
% Wyświetlenie macierzy transformacji na symbolach
for i = 1:7
    fprintf('Macierz transformacji T%d%d na symbolach:\n', i-1, i);
    disp((T_syms(:, :, i)));
end
%}



% Macierze transformacji na symbolach
T01_syms = T_syms(:,:,1);
T12_syms = T_syms(:,:,2);
T23_syms = T_syms(:,:,3);
T34_syms = T_syms(:,:,4);
T45_syms = T_syms(:,:,5);
T56_syms = T_syms(:,:,6);
T6k_syms = T_syms(:,:,7);

T02_syms = T01_syms*T12_syms;
T03_syms = T01_syms*T12_syms*T23_syms;
T04_syms = T01_syms*T12_syms*T23_syms*T34_syms;
T05_syms = T01_syms*T12_syms*T23_syms*T34_syms*T45_syms;
T06_syms = T01_syms*T12_syms*T23_syms*T34_syms*T45_syms*T56_syms;
T0k_syms = T01_syms*T12_syms*T23_syms*T34_syms*T45_syms*T56_syms*T6k_syms;

% Macierze transformacji na wartościach

T01 = double(subs(T01_syms, [q1(t), a0, d1], [q(1), a(1), d(1)]));
T12 = double(subs(T12_syms, [q2(t), a1, d2], [q(2), a(2), d(2)]));
T23 = double(subs(T23_syms, [q3(t), a2, d3], [q(3), a(3), d(3)]));
T34 = double(subs(T34_syms, [q4(t), a3, d4], [q(4), a(4), d(4)]));
T45 = double(subs(T45_syms, [q5(t), a4, d5], [q(5), a(5), d(5)]));
T56 = double(subs(T56_syms, [q6(t), a5, d6], [q(6), a(6), d(6)])); 
T6k = double(subs(T6k_syms, [dk], [d(7)])); 

T02 = double(subs(T02_syms, [q1(t), q2(t), a0, a1, d1, d2], [q(1), q(2), a(1), a(2), d(1), d(2)]));
T03 = double(subs(T03_syms, [q1(t), q2(t), q3(t), a0, a1, a2, d1, d2, d3], [q(1), q(2), q(3), a(1), a(2), a(3), d(1), d(2), d(3)]));
T04 = double(subs(T04_syms, [q1(t), q2(t), q3(t), q4(t), a0, a1, a2, a3, d1, d2, d3, d4], [q(1), q(2), q(3), q(4), a(1), a(2), a(3), a(4), d(1), d(2), d(3), d(4)]));
T05 = double(subs(T05_syms, [q1(t), q2(t), q3(t), q4(t), q5(t), a0, a1, a2, a3, a4, d1, d2, d3, d4, d5], [q(1), q(2), q(3), q(4), q(5), a(1), a(2), a(3), a(4), a(5), d(1), d(2), d(3), d(4), d(5)]));
T06 = double(subs(T06_syms, [q1(t), q2(t), q3(t), q4(t), q5(t), q6(t), a0, a1, a2, a3, a4, a5, d1, d2, d3, d4, d5, d6], [q(1), q(2), q(3), q(4), q(5), q(6), a(1), a(2), a(3), a(4), a(5), a(6), d(1), d(2), d(3), d(4), d(5), d(6)]));
T0k = double(subs(T0k_syms, [q1(t), q2(t), q3(t), q4(t), q5(t), q6(t), a0, a1, a2, a3, a4, a5, d1, d2, d3, d4, d5, d6, dk], [q(1), q(2), q(3), q(4), q(5), q(6), a(1), a(2), a(3), a(4), a(5), a(6), d(1), d(2), d(3), d(4), d(5), d(6), d(7)]));

% Wyswietlanie wszystkich macierzy na symbolach
%{
disp('T01_syms =');
disp(T01_syms);
disp('T12_syms =');
disp(T12_syms);
disp('T23_syms =');
disp(T23_syms);
disp('T34_syms =');
disp(T34_syms);
disp('T45_syms =');
disp(T45_syms);
disp('T56_syms =');
disp(T56_syms);
disp('T6k_syms =');
disp(T6k_syms);
%}


%{
disp('T02_syms =');
disp(T02_syms);
disp('T03_syms =');
disp(T03_syms);
disp('T04_syms =');
disp(T04_syms);
disp('T05_syms =');
disp(T05_syms);
disp('T06_syms =');
disp(T06_syms);
disp('T0k_syms =');
disp(T0k_syms);
%}

% Wyswietlanie wszystkich macierzy na liczbach
%{
disp('T01 =');
disp(T01);
disp('T12 =');
disp(T12);
disp('T23 =');
disp(T23);
disp('T34 =');
disp(T34);
disp('T45 =');
disp(T45);
disp('T56 =');
disp(T56);
disp('T6k =');
disp(T6k);
%}

%{
disp('T02 =');
disp(T02);
disp('T03 =');
disp(T03);
disp('T04 =');
disp(T04);
disp('T05 =');
disp(T05);
disp('T06 =');
disp(T06);
disp('T0k =');
disp(T0k);
%}


% Wyznaczenie katow Eulera z kosinusow kierunkowych - rotacja ZYZ

% Wydzielenie macierzy rotacji R0k (3x3)
R0k = T0k(1:3, 1:3);
%{
disp('Macierz rotacji R0k:');
disp(R0k);
%}

% Wydzielenie wektora położenia P0k (3x1)
P0k = T0k(1:3, 4);
%{
disp('Wektor położenia P0k:');
disp(P0k);
%}


Eulers = ToEulers(T0k);


% Duza macierz T, do przesylania wszystkich macierzy transformacji
T = zeros(4, 4, 7);

T(:,:,1) = T01;
T(:,:,2) = T02;
T(:,:,3) = T03;
T(:,:,4) = T04;
T(:,:,5) = T05;
T(:,:,6) = T06;
T(:,:,7) = T0k;

T_syms(:,:,1) = T01_syms;
T_syms(:,:,2) = T02_syms;
T_syms(:,:,3) = T03_syms;
T_syms(:,:,4) = T04_syms;
T_syms(:,:,5) = T05_syms;
T_syms(:,:,6) = T06_syms;
T_syms(:,:,7) = T0k_syms;

% Wyświetla wymiary T
%disp(size(T)); 

% Return
Eulers = Eulers;
T = T;
T_syms = T_syms;
end