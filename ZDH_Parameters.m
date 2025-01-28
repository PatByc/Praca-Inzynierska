function [] = ZDH_Parameters(alpha, a, d)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% katy alpha = [alpha0, alpha1, alpha2, alpha3, alpha4, alpha5];
% dlugosci a = [a0, a1, a2, a3, a4, a5];
% zmienne konfiguracyjne q = [q1(t), q2(t), q3(t), q4(t), q5(t), q6(t)];
% przesuniecia d = [d1, d2, d3, d4, d5, d6];


% Symboliczne zmienne
syms t q1(t) q2(t) q3(t) q4(t) q5(t) q6(t); % Zmienne konfiguracyjne jako funkcje czasu
syms a0 a1 a2 a3 a4 a5;                     % Długości członów
syms d1 d2 d3 d4 d5 d6;                     % Przesunięcia wzdłuż osi Z
syms alpha0 alpha1 alpha2 alpha3 alpha4 alpha5; % Kąty między osiami Z

% Wektory parametrow ZDH

% katy alpha = [alpha0, alpha1, alpha2, alpha3, alpha4, alpha5];
alpha = [sym(alpha(1)), sym(alpha(2)), sym(alpha(3)), sym(alpha(4)), sym(alpha(5)), sym(alpha(6))];


% Symboliczna macierz ZDH dla 6 przegubów - [alpha_i_1 / a_i_1 / theta_i / d_i]
ZDH = [
    alpha0,   a0,  q1(t),  d1;  % Oś 1
    alpha1,   a1,  q2(t),  d2;  % Oś 2
    alpha2,   a2,  q3(t),  d3;  % Oś 3
    alpha3,   a3,  q4(t),  d4;  % Oś 4
    alpha4,   a4,  q5(t),  d5;  % Oś 5
    alpha5,   a5,  q6(t),  d6;  % Oś 6
];

disp('Konstrukcja macierzy ZDH: ');
disp(ZDH);

% Podstawienie rzeczywistych wartości do macierzy ZDH
ZDH_with_values = subs(ZDH, ...
    [alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, ... % Symbole alpha
     a0, a1, a2, a3, a4, a5, ...                        % Symbole a
     d1, d2, d3, d4, d5, d6], ...                      % Symbole d
    [alpha(1), alpha(2), alpha(3), alpha(4), alpha(5), alpha(6), ... % Wartości alpha
     a(1), a(2), a(3), a(4), a(5), a(6), ...                          % Wartości a
     d(1), d(2), d(3), d(4), d(5), d(6)]);                           % Wartości d

disp('Macierz ZDH z podstawionymi paramtrami: ');
disp(ZDH_with_values);




% Return parameters

end