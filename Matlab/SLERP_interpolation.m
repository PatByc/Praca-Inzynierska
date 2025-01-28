function [T_interp] = SLERP_interpolation(T_P1, T_P2, points_num)

% Liczba punktów interpolowanych
num_points = points_num;

% Inicjalizacja macierzy wynikowej
T_interp = zeros(4, 4, 7, num_points);

% Pętla po liczbie punktów interpolowanych
for i = 1:num_points
    t = i / (num_points + 1); % Wartość t dla aktualnego punktu
    T_interp(:,:,:,i) = interpolate_point(T_P1, T_P2, t);
end

end

function T_interp = interpolate_point(T_P1, T_P2, t)
    % Ekstrakcja i interpolacja
    
    % Ekstrakcja macierzy transformacji
    num_matrices = size(T_P1, 3);
    T_interp = zeros(4, 4, num_matrices);
    
    for i = 1:num_matrices
        % Ekstrakcja macierzy rotacji i wektorów pozycji
        R_P1 = T_P1(1:3, 1:3, i);
        R_P2 = T_P2(1:3, 1:3, i);
        P_P1 = T_P1(1:3, 4, i);
        P_P2 = T_P2(1:3, 4, i);

        % Konwersja do kwaternionów
        q_P1 = rotationMatrixToQuaternion(R_P1);
        q_P2 = rotationMatrixToQuaternion(R_P2);
       
        % Interpolacja kwaternionów
        q_interp = slerp(q_P1, q_P2, t);
        
        % Konwersja z powrotem do macierzy rotacji
        R_interp = quaternionToRotationMatrix(q_interp);

        % Interpolacja wektora pozycji
        P_interp = (1 - t) * P_P1 + t * P_P2;
        
        % Zapisanie wyniku do macierzy transformacji
        T_interp(1:3, 1:3, i) = R_interp;
        T_interp(1:3, 4, i) = P_interp;
        T_interp(4, :, i) = [0, 0, 0, 1]; % Jednorodna reprezentacja

    end
end

function q = rotationMatrixToQuaternion(R)
    qw = sqrt(1 + R(1,1) + R(2,2) + R(3,3)) / 2;
    qx = (R(3,2) - R(2,3)) / (4 * qw);
    qy = (R(1,3) - R(3,1)) / (4 * qw);
    qz = (R(2,1) - R(1,2)) / (4 * qw);
    q = [qw, qx, qy, qz];
end

function q_interp = slerp(q1, q2, t)
    % Oblicz iloczyn skalarny
    dot_product = dot(q1, q2);
    
    % Korekcja dla orientacji przeciwnych
    if dot_product < 0
        q1 = -q1;
        dot_product = -dot_product;
    end

    % Jeżeli kąty są bardzo bliskie, wykonaj interpolację liniową
    if dot_product > 0.9995
        q_interp = (1 - t) * q1 + t * q2;
        q_interp = q_interp / norm(q_interp);
        return;
    end
    
    % Oblicz kąt pomiędzy kwaternionami
    theta_0 = acos(dot_product);
    theta = theta_0 * t;
    
    % Oblicz wektor ortogonalny
    q3 = q2 - q1 * dot_product;
    q3 = q3 / norm(q3);


    % Interpolacja kwaternionu
    q_interp = q1 * cos(theta) + q3 * sin(theta);
    
end

function R = quaternionToRotationMatrix(q)
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    R = [
        1 - 2*qy^2 - 2*qz^2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy;
        2*qx*qy + 2*qw*qz, 1 - 2*qx^2 - 2*qz^2, 2*qy*qz - 2*qw*qx;
        2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx^2 - 2*qy^2
    ];
end
