#include <math.h>
#include <stdio.h> 
#include <float.h>
#include <stddef.h>
#include <string.h>
#include "Robotics_Calculations.h"

#define M_PI 3.14159265358979323846

void multiplyMatrix_4x4(const float A[4][4], const float B[4][4], float C[4][4]){
     for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            C[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void SavePath(float q_results[10][7], float q_P1[7], float q_P2[7], float q_P3[7],
                float q_P4[7], float q_P5[7], float q_P6[7], float q_P7[7], float q_P8[7], float q_P9[7], float q_P10[7]){

     // Wypełnianie q_results przez powielanie q_PX
    for (int col = 0; col < 7; col++) {
        q_results[0][col] = q_P1[col];
        q_results[1][col] = q_P2[col];
        q_results[2][col] = q_P3[col];
        q_results[3][col] = q_P4[col];
        q_results[4][col] = q_P5[col];
        q_results[5][col] = q_P6[col];
        q_results[6][col] = q_P7[col];
        q_results[7][col] = q_P8[col];
        q_results[8][col] = q_P9[col];
        q_results[9][col] = q_P10[col];
    }
    
}

void transposeMatrix_3x3(const float matrix[3][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[j][i] = matrix[i][j];
        }
    }
}


void multiplyMatrixVector_3x1(float matrix[3][3], float vector[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0; // Inicjalizacja wyniku
        for (int j = 0; j < 3; j++) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}

void multiplyMatrixVector_4x1(float matrix[4][4], float vector[4], float result[4]) {
    for (int i = 0; i < 4; i++) {
        result[i] = 0; // Inicjalizacja wyniku
        for (int j = 0; j < 4; j++) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}

void matrixExtractionT(const float Txxi[4][4][7], int index, float Txx[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Txx[i][j] = Txxi[i][j][index];
        }
    }
}

void matrixExtractionT_interp(const float T_interp[4][4][7][10], int index, float T_P2[4][4][7]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 7; k++) {
                T_P2[i][j][k] = T_interp[i][j][k][index];
            }
        }
    }
}

void matrixExtractionR(const float T[4][4], float R[3][3]){
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = T[i][j];
        }
    }
}

void vectorExtractionT(const float T[4][4], float P[3]){
    for (int i = 0; i < 3; i++) {
        P[i] = T[i][3];
    }
}


void Calculate_R03_x(float q1_x, float q2_x, float q3_x, float R03_x[3][3]) {
    // Wiersz 1
    R03_x[0][0] = cos(q1_x) * cos(q2_x) * cos(q3_x) - cos(q1_x) * sin(q2_x) * sin(q3_x);
    R03_x[0][1] = -cos(q1_x) * cos(q2_x) * sin(q3_x) - cos(q1_x) * cos(q3_x) * sin(q2_x);
    R03_x[0][2] = -sin(q1_x);

    // Wiersz 2
    R03_x[1][0] = cos(q2_x) * cos(q3_x) * sin(q1_x) - sin(q1_x) * sin(q2_x) * sin(q3_x);
    R03_x[1][1] = -cos(q2_x) * sin(q1_x) * sin(q3_x) - cos(q3_x) * sin(q1_x) * sin(q2_x);
    R03_x[1][2] = cos(q1_x);

    // Wiersz 3
    R03_x[2][0] = -cos(q2_x) * sin(q3_x) - cos(q3_x) * sin(q2_x);
    R03_x[2][1] = sin(q2_x) * sin(q3_x) - cos(q2_x) * cos(q3_x);
    R03_x[2][2] = 0.0f;
}

void PrintPath(float q_results[10][7], const char* path_name){
    printf("%s :\n", path_name);
    for (int row = 0; row < 10; row++) {
        printf("P_%d: ", row+1);
        for (int col = 0; col < 7; col++) {
            printf("%.4f ", q_results[row][col]);
        }
        printf("\n");
    }
}

void multiplyMatrix_3x3(const float A[3][3], const float B[3][3], float C[3][3]){
     for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Funkcja do stosowania tolerancji na macierzy 3x3 potrzebna do poprawnosci atan2
void toleranceMatrix(float matrix[3][3]) {
    const float tolerance = 1e-6;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (fabs(matrix[i][j]) < tolerance) {
                matrix[i][j] = 0.0f; // Zaokrąglenie wartości do zera
            }
            // Jeśli element to dokładnie 0, dodaj sztuczną wartość
            if (matrix[i][j] == 0.0f) {
                matrix[i][j] += 0.00000000000000000000001f;
            }
        }
    }
}

// Funkcja do wyświetlania wektorów
void printVector(const float vector[], int size, const char* name) {
    printf("%s: ", name);
    for (int i = 0; i < size; i++) {
        printf("%.4f ", vector[i]);
    }
    printf("\n");
}


// Funkcja do wyświetlania danej macierzy transformacji
void printMatrixT_4x4(const float matrix[4][4], const char* name) {
    printf("%s:\n", name);
    for (int i = 0; i < 4; i++) {        
        for (int j = 0; j < 4; j++) {
            printf("%.4f ", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
} 

void printMatrix_3x3(const float matrix[3][3], const char* name) {
    printf("%s:\n", name);
    for (int i = 0; i < 3; i++) {        
        for (int j = 0; j < 3; j++) {
            printf("%.4f ", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

// Funkcja do wyświetlania danej macierzy transformacji
void printMatrixT_4x4x7(const float matrix[4][4][7], int num_matrices) {
    int k = num_matrices;
    printf("T%d%d:\n", k, k + 1);
    for (int i = 0; i < 4; i++) {        
        for (int j = 0; j < 4; j++) {
            printf("%.4f ", matrix[i][j][k]);
        }
        printf("\n");
    }
    printf("\n");
} 

// Funkcja do wyświetlania macierzy transformacji 4x4x7
void printAllMatrixT_4x4x7(const float matrix[4][4][7]) {
    for (int k = 0; k < 7; k++) {
        printf("T0%d:\n", k + 1);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                printf("%.4f ", matrix[i][j][k]);
            }
            printf("\n");
        }
        printf("\n");
    }
}

void printConfigsRad(const float configs[8][7], int rows, int cols) {
    printf("Configs (in radians):\n");
    for (int i = 0; i < rows; i++) {
        printf("Config %d: ", i + 1);
        for (int j = 0; j < cols; j++) {
            printf("%.4f ", configs[i][j]);
        }
        printf("\n");
    }
}

#define RAD_TO_DEG 57.2957795 // Stała do konwersji radianów na stopnie (180 / pi)

void printConfigsDeg(const float configs[8][7], int rows, int cols) {
    printf("Configs (in degrees):\n");
    for (int i = 0; i < rows; i++) {
        printf("Config %d: ", i + 1);
        for (int j = 0; j < cols; j++) {
            printf("%.2f ", configs[i][j] * RAD_TO_DEG);
        }
        printf("\n");
    }
}

void ToEulers(float T0k[4][4], float Eulers[6]) {
    // Pozycja P0k
    float P0k[3] = {T0k[0][3], T0k[1][3], T0k[2][3]};

    // Rotacja R0k
    float R0k[3][3];
    matrixExtractionR(T0k, R0k);
    //printMatrix_3x3(R0k, "R0k");
    
    //printMatrix_3x3(R0k, "przed");
    //toleranceMatrix(R0k);
    //printMatrix_3x3(R0k, "po");

    // Obliczenie pitch
    float pitch = atan2(sqrt((R0k[0][2] * R0k[0][2]) + (R0k[1][2] * R0k[1][2])), R0k[2][2]);

    // Obliczenie roll
    float roll = atan2(R0k[1][2], R0k[0][2]);

    // Obliczenie yaw
    float yaw = atan2(R0k[2][1], -R0k[2][0]);


    // Przypisanie do wyniku Eulers (pozycja + orientacja)
    Eulers[0] = P0k[0]; // Pozycja X
    Eulers[1] = P0k[1]; // Pozycja Y
    Eulers[2] = P0k[2]; // Pozycja Z
    Eulers[3] = roll;   // Orientacja roll
    Eulers[4] = pitch;  // Orientacja pitch
    Eulers[5] = yaw;    // Orientacja yaw
}

void ForwardKinematics(const float alpha[], const float a[], const float q[], const float d[], 
                         float Eulers[], float T[4][4][7]){


    // Tworzenie macierzy transformacji
    for (int i = 0; i < 7; ++i) {
        T[0][0][i] = cos(q[i]);
        T[0][1][i] = -sin(q[i]);
        T[0][2][i] = 0;
        T[0][3][i] = a[i];

        T[1][0][i] = sin(q[i]) * cos(alpha[i]);
        T[1][1][i] = cos(q[i]) * cos(alpha[i]);
        T[1][2][i] = -sin(alpha[i]);
        T[1][3][i] = -d[i] * sin(alpha[i]);

        T[2][0][i] = sin(q[i]) * sin(alpha[i]);
        T[2][1][i] = cos(q[i]) * sin(alpha[i]);
        T[2][2][i] = cos(alpha[i]);
        T[2][3][i] = d[i] * cos(alpha[i]);

        T[3][0][i] = 0;
        T[3][1][i] = 0;
        T[3][2][i] = 0;
        T[3][3][i] = 1;
    }

    // Macierze transformacji na wartościach
    float T01[4][4], T02[4][4], T03[4][4], T04[4][4], T05[4][4], T06[4][4], T0k[4][4];
    float T12[4][4], T23[4][4], T34[4][4], T45[4][4], T56[4][4], T6k[4][4];

    matrixExtractionT(T, 0, T01); // Wyciagniecie macierzy T01 z T
    matrixExtractionT(T, 1, T12); // Wyciagniecie macierzy T12 z T
    matrixExtractionT(T, 2, T23); 
    matrixExtractionT(T, 3, T34); 
    matrixExtractionT(T, 4, T45); 
    matrixExtractionT(T, 5, T56); 
    matrixExtractionT(T, 6, T6k); 

    // Obliczenia kolejnych macierzy
    multiplyMatrix_4x4(T01, T12, T02); // T01 * T12 = T02
    multiplyMatrix_4x4(T02, T23, T03); // T02 * T23 = T03
    multiplyMatrix_4x4(T03, T34, T04); // T03 * T34 = T04
    multiplyMatrix_4x4(T04, T45, T05); // T04 * T45 = T05
    multiplyMatrix_4x4(T05, T56, T06); // T05 * T56 = T06
    multiplyMatrix_4x4(T06, T6k, T0k); // T06 * T6k = T0k
   
    // Przepisanie macierzy do wyniku T
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            T[row][col][0] = T01[row][col];
            T[row][col][1] = T02[row][col];
            T[row][col][2] = T03[row][col];
            T[row][col][3] = T04[row][col];
            T[row][col][4] = T05[row][col];
            T[row][col][5] = T06[row][col];
            T[row][col][6] = T0k[row][col];
        }
    }
    

    ToEulers(T0k, Eulers);
}


void InverseKinematics(const float a[], const float d[], const float T[4][4][7], 
                         const float q_actual[7], float q_out[7]) {


    // Ekstrakcja macierzy transformacji T04
    float T04[4][4];
    matrixExtractionT(T, 3, T04);

    // Kinematyka przestrzenna - obliczanie q1, q2, q3

    // q1

    // Punkt P04 1x3 (Punkt w ukladzie 4 - q4) 
    float P04_3[3] = {T04[0][3], T04[1][3], T04[2][3]};

    // Wyznaczenie q1_A i q1_B
    float alpha1 = atan2(P04_3[1], P04_3[0]);

    float sin_beta1 = d[2] / sqrt(P04_3[0] * P04_3[0] + P04_3[1] * P04_3[1]);
    float cos_beta1 = sqrt(1 - sin_beta1 * sin_beta1);

    float beta1_positive = atan2(sin_beta1, cos_beta1);
    float beta1_negative = atan2(sin_beta1, -cos_beta1);

    float q1_A = alpha1 - beta1_positive;
    float q1_B = alpha1 - beta1_negative;

    if (fabs(P04_3[0]) < 1e-6 && fabs(P04_3[1]) < 1e-6) {
        // Punkt docelowy leży na osi, kąt q1 jest nieokreślony
        q1_A = 0;
        q1_B = 0;
    }

    // q3

    // Wzor: P14 = ((T01(q1))^-1 * P04  

    // Zmiana P04 1x3 na homogeniczne współrzędne, dodanie jedynki jako czwartego elementu 1x4
    float P04[4] = {P04_3[0], P04_3[1], P04_3[2], 1.0};

    // Transformacje dla q1_A i q1_B
    float T01_q1_A[4][4] = {
        {cos(q1_A), -sin(q1_A), 0, 0},
        {sin(q1_A),  cos(q1_A), 0, 0},
        {0,                0,   1, d[0]},
        {0,                0,   0, 1}
    };

    float T01_q1_B[4][4] = {
        {cos(q1_B), -sin(q1_B), 0, 0},
        {sin(q1_B),  cos(q1_B), 0, 0},
        {0,                0,   1, d[0]},
        {0,                0,   0, 1}
    };

    
    // Wyznaczenie P14_q1_A

    // Potrzebne P01 oraz R01 z q1_A
    float P01_q1_A[3];
    vectorExtractionT(T01_q1_A, P01_q1_A);

    float R01_q1_A[3][3];
    matrixExtractionR(T01_q1_A, R01_q1_A);

    // Transpozycja macierzy R01_q1_A
    float R01_q1_A_T[3][3];
    transposeMatrix_3x3(R01_q1_A, R01_q1_A_T);

    float P01_q1_A_inv[3];
    multiplyMatrixVector_3x1(R01_q1_A_T, P01_q1_A, P01_q1_A_inv);

    float T01_q1_A_inv[4][4] = {
        {R01_q1_A_T[0][0], R01_q1_A_T[0][1], R01_q1_A_T[0][2], - P01_q1_A_inv[0]},
        {R01_q1_A_T[1][0], R01_q1_A_T[1][1], R01_q1_A_T[1][2], - P01_q1_A_inv[1]},
        {R01_q1_A_T[2][0], R01_q1_A_T[2][1], R01_q1_A_T[2][2], - P01_q1_A_inv[2]},
        {0,                               0,                0, 1}
    };

    float P14_q1_A[4];
    multiplyMatrixVector_4x1(T01_q1_A_inv, P04, P14_q1_A);


    // Wyznaczenie P14_q1_B

    // Potrzebne P01 oraz R01 z q1_B
    float P01_q1_B[3];
    vectorExtractionT(T01_q1_B, P01_q1_B);

    float R01_q1_B[3][3];
    matrixExtractionR(T01_q1_B, R01_q1_B);

    // Transpozycja macierzy R01_q1_B
    float R01_q1_B_T[3][3];
    transposeMatrix_3x3(R01_q1_B, R01_q1_B_T);

    float P01_q1_B_inv[3];
    multiplyMatrixVector_3x1(R01_q1_B_T, P01_q1_B, P01_q1_B_inv);

    float T01_q1_B_inv[4][4] = {
        {R01_q1_B_T[0][0], R01_q1_B_T[0][1], R01_q1_B_T[0][2], - P01_q1_B_inv[0]},
        {R01_q1_B_T[1][0], R01_q1_B_T[1][1], R01_q1_B_T[1][2], - P01_q1_B_inv[1]},
        {R01_q1_B_T[2][0], R01_q1_B_T[2][1], R01_q1_B_T[2][2], - P01_q1_B_inv[2]},
        {0,                               0,                0, 1}
    };

    float P14_q1_B[4];
    multiplyMatrixVector_4x1(T01_q1_B_inv, P04, P14_q1_B);



    // Wartości geometryczne dla P14
    float x_A = P14_q1_A[0];
    float z_A = P14_q1_A[2];

    float x_B = P14_q1_B[0];
    float z_B = P14_q1_B[2];

    // Standardowy przypadek dla q1_A
    float cos_beta_q1_A = (x_A * x_A + z_A * z_A - a[2] * a[2] - d[3] * d[3]) / (2 * a[2] * d[3]);
    cos_beta_q1_A = fmaxf(-1, fminf(1, cos_beta_q1_A)); // Ograniczenie wartości do zakresu [-1, 1]

    float beta_q3_A = acos(cos_beta_q1_A); // beta > 0 dla q1_A
    float beta_q3_B = -acos(cos_beta_q1_A); // beta < 0 dla q1_A

    float q3_A = M_PI / 2 + beta_q3_A;
    float q3_B = M_PI / 2 + beta_q3_B;

    // Standardowy przypadek dla q1_B

    float cos_beta_q1_B = (x_B * x_B + z_B * z_B - a[2] * a[2] - d[3] * d[3]) / (2 * a[2] * d[3]);
    cos_beta_q1_B = fmaxf(-1, fminf(1, cos_beta_q1_B)); // Ograniczenie wartości do zakresu [-1, 1]
    
    float beta_q3_C = acos(cos_beta_q1_B); // beta > 0 dla q1_B
    float beta_q3_D = -acos(cos_beta_q1_B); // beta > 0 dla q1_B

    float q3_C = M_PI / 2 + beta_q3_C;
    float q3_D = M_PI / 2 + beta_q3_D;

    // q2

    // Obliczenie α dla q1_A
    float alpha2_A = atan2(P14_q1_A[2], P14_q1_A[0]);

    // Wyznaczenie cos(beta) dla q1_A
    float r_A = sqrt(P14_q1_A[0] * P14_q1_A[0] + P14_q1_A[2] * P14_q1_A[2]);
    float cos_beta_q2_A = (P14_q1_A[0] * P14_q1_A[0] + P14_q1_A[2] * P14_q1_A[2] + a[2] * a[2] - d[3] * d[3]) / (2 * a[2] * r_A);
    
    // Ograniczenie wartości do przedziału [-1, 1]
    cos_beta_q2_A = fmaxf(-1, fminf(1, cos_beta_q2_A));

    // Obliczenie beta dla q2_A
    float beta_q2_A = acos(cos_beta_q2_A);
    float beta_q2_B = acos(cos_beta_q2_A);

    // Wyznaczenie q2 dla q1_A - Warunek sprawdzajacy znak beta (roznorodnosc rozwiazan)
    float q2_A, q2_B;

    if (beta_q3_B < 0){
        q2_A = - (alpha2_A + beta_q2_A); // Negacja kąta, bo jest on obliczany w złym kierunku
    } else if (beta_q3_B == 0){
        q2_A = - (alpha2_A + beta_q2_A);
    }
    else{
        q2_A = 0;
    }

   
    if (beta_q3_A > 0){
        q2_B = - (alpha2_A - beta_q2_B); // Negacja kąta, bo jest on obliczany w złym kierunku
    } else if (beta_q3_A == 0)
        q2_B = - (alpha2_A - beta_q2_B); 
    else{
        q2_B = 0;
    }

    // Obliczenie α dla q1_B
    float alpha2_B = atan2(P14_q1_B[2], P14_q1_B[0]); 

    // Wyznaczenie cos(beta) dla q1_B
    float r_B = sqrt(P14_q1_B[2] * P14_q1_B[2]  + P14_q1_B[0] * P14_q1_B[0]);
    float cos_beta_q2_B = ((P14_q1_B[0] * P14_q1_B[0]) + (P14_q1_B[2] * P14_q1_B[2] + a[2] * a[2] - d[3] * d[3])) / (2 * a[2] * r_B);

    // Ograniczenie wartości do przedziału [-1, 1]
    cos_beta_q2_B = fmaxf(-1, fminf(1, cos_beta_q2_B));

    // Obliczenie beta dla q1_B
    float beta_q2_C = acos(cos_beta_q2_B);
    float beta_q2_D = acos(cos_beta_q2_B);

    // Wyznaczenie q2 dla q1_B - Warunek sprawdzajacy znak beta (roznorodnosc rozwiazan)
    float q2_C, q2_D;

    if (beta_q3_D < 0){
        q2_C = - (alpha2_B + beta_q2_C); // Negacja kąta, bo jest on obliczany w złym kierunku
    } else if (beta_q3_D == 0){
        q2_C = - (alpha2_B + beta_q2_C);
    }
    else{
        q2_C = 0;
    }

   
    if (beta_q3_C > 0){
        q2_D = - (alpha2_B - beta_q2_D); // Negacja kąta, bo jest on obliczany w złym kierunku
    } else if (beta_q3_C == 0)
        q2_D = - (alpha2_B - beta_q2_D); 
    else{
        q2_D = 0;
    }

    // ---------------------------------------------------------------------------

    // Kinematyka orientacyjna - obliczanie q4, q5, q6

    // Ekstrakcja macierzy transformacji T0k
    float T0k[4][4];
    matrixExtractionT(T, 6, T0k);

    // Ekstrakcja macierzy transformacji T03
    float T03[4][4];
    matrixExtractionT(T, 2, T03);

    // R03 * R3k = R0k => R3k = (RO3)^-1 * R0k => R3k = (R03)^T * R0k
    float R0k[3][3];
    matrixExtractionR(T0k, R0k);

    // Wyznaczanie 4 roznych brakujacych macierzy R03
   
    float R03_A[3][3];
    Calculate_R03_x(q1_A, q2_A, q3_A, R03_A);

    float R03_B[3][3];
    Calculate_R03_x(q1_A, q2_B, q3_B, R03_B);

    float R03_C[3][3];
    Calculate_R03_x(q1_B, q2_C, q3_C, R03_C);

    float R03_D[3][3];
    Calculate_R03_x(q1_B, q2_D, q3_D, R03_D);
    
    // Wyznaczenie R3k

    float R03_A_T[3][3];
    transposeMatrix_3x3(R03_A, R03_A_T); // (R03_A)^T

    float R03_B_T[3][3];
    transposeMatrix_3x3(R03_B, R03_B_T);

    float R03_C_T[3][3];
    transposeMatrix_3x3(R03_C, R03_C_T);

    float R03_D_T[3][3];
    transposeMatrix_3x3(R03_D, R03_D_T);

    float R3k_A[3][3];
    multiplyMatrix_3x3(R03_A_T, R0k, R3k_A); // (R03_A_T) * R0k = R3k_A 

    float R3k_B[3][3];
    multiplyMatrix_3x3(R03_B_T, R0k, R3k_B);

    float R3k_C[3][3];
    multiplyMatrix_3x3(R03_C_T, R0k, R3k_C);

    float R3k_D[3][3];
    multiplyMatrix_3x3(R03_D_T, R0k, R3k_D);

    // q5 (8 rozwiazan)

    // Zmienna sq5 i cq5 dla różnych konfiguracji
    float sq5_A, cq5_A, q5_A, q5_B;
    float sq5_B, cq5_B, q5_C, q5_D;
    float sq5_C, cq5_C, q5_E, q5_F;
    float sq5_D, cq5_D, q5_G, q5_H;

    // z R03_A
    sq5_A = sqrt((R3k_A[0][2] * R3k_A[0][2]) + (R3k_A[2][2] * R3k_A[2][2]));
    cq5_A = -R3k_A[1][2];

    q5_A = atan2(sq5_A, cq5_A);
    q5_B = -atan2(sq5_A, cq5_A);

    // z R03_B
    sq5_B = sqrt((R3k_B[0][2] * R3k_B[0][2]) + (R3k_B[2][2] * R3k_B[2][2]));
    cq5_B = -R3k_B[1][2];

    q5_C = atan2(sq5_B, cq5_B);
    q5_D = -atan2(sq5_B, cq5_B);

    // z R03_C
    sq5_C = sqrt((R3k_C[0][2] * R3k_C[0][2]) + (R3k_C[2][2] * R3k_C[2][2]));
    cq5_C = -R3k_C[1][2];

    q5_E = atan2(sq5_C, cq5_C);
    q5_F = -atan2(sq5_C, cq5_C);

    // z R03_D
    sq5_D = sqrt((R3k_D[0][2] * R3k_D[0][2]) + (R3k_D[2][2] * R3k_D[2][2]));
    cq5_D = -R3k_D[1][2];

    q5_G = atan2(sq5_D, cq5_D);
    q5_H = -atan2(sq5_D, cq5_D);

    // q4 (8 rozwiazan)

    // Zmienne dla q4
    float q4_A, q4_B, q4_C, q4_D, q4_E, q4_F, q4_G, q4_H;

    // q4 dla R03_A
    q4_A = atan2(R3k_A[2][2], R3k_A[0][2]);
    q4_B = atan2(-R3k_A[2][2], -R3k_A[0][2]);

    // q4 dla R03_B
    q4_C = atan2(R3k_B[2][2], R3k_B[0][2]);
    q4_D = atan2(-R3k_B[2][2], -R3k_B[0][2]);

    // q4 dla R03_C
    q4_E = atan2(R3k_C[2][2], R3k_C[0][2]);
    q4_F = atan2(-R3k_C[2][2], -R3k_C[0][2]);

    // q4 dla R03_D
    q4_G = atan2(R3k_D[2][2], R3k_D[0][2]);
    q4_H = atan2(-R3k_D[2][2], -R3k_D[0][2]);

    // q6 (8 rozwiazan)

    // Zmienne dla q6
    float q6_A, q6_B, q6_C, q6_D, q6_E, q6_F, q6_G, q6_H;

    // q6 dla R03_A
    q6_A = atan2(R3k_A[1][1], R3k_A[1][0]);
    q6_B = atan2(-R3k_A[1][1], -R3k_A[1][0]);

    // q6 dla R03_B
    q6_C = atan2(R3k_B[1][1], R3k_B[1][0]);
    q6_D = atan2(-R3k_B[1][1], -R3k_B[1][0]);

    // q6 dla R03_C
    q6_E = atan2(R3k_C[1][1], R3k_C[1][0]);
    q6_F = atan2(-R3k_C[1][1], -R3k_C[1][0]);

    // q6 dla R03_D
    q6_G = atan2(R3k_D[1][1], R3k_D[1][0]);
    q6_H = atan2(-R3k_D[1][1], -R3k_D[1][0]);

    // Konfiguracje
    float configs[8][7] = {
        {q1_A, q2_A, q3_A, q4_A, q5_A, q6_A, 0},
        {q1_A, q2_A, q3_A, q4_B, q5_B, q6_B, 0},
        {q1_A, q2_B, q3_B, q4_C, q5_C, q6_C, 0},
        {q1_A, q2_B, q3_B, q4_D, q5_D, q6_D, 0},
        {q1_B, q2_C, q3_C, q4_E, q5_E, q6_E, 0},
        {q1_B, q2_C, q3_C, q4_F, q5_F, q6_F, 0},
        {q1_B, q2_D, q3_D, q4_G, q5_G, q6_G, 0},
        {q1_B, q2_D, q3_D, q4_H, q5_H, q6_H, 0}
    };

    printConfigsRad(configs, 8, 7);
    //printConfigsDeg(configs, 8, 7);

    // Inicjalizacja minimalnej różnicy i preferowanej konfiguracji
    int num_configs = 8;
    int num_joints = 7;
    float min_diff = FLT_MAX;
    int preferred_index = -1; // Numer wybranej konfiguracji
    float preferred_config[7] = {0}; // Wybrana konfiguracja

    // Kryterium tolerancji
    for (int i = 0; i < num_configs; i++) {
        float diff = 0.0;

        // Obliczanie różnicy (norma euklidesowa)
        for (int j = 0; j < num_joints; j++) {
            diff += pow(configs[i][j] - q_actual[j], 2);
        }
        diff = sqrt(diff);

        // Aktualizacja minimalnej różnicy i preferowanej konfiguracji
        if (diff < min_diff) {
            min_diff = diff;
            preferred_index = i;

            // Kopiowanie konfiguracji
            for (int j = 0; j < num_joints; j++) {
                preferred_config[j] = configs[i][j];
            }
        }
    }

    for (int i = 0; i < num_joints; i++) {
        q_out[i] = preferred_config[i];
    }

    // Wyświetlenie wyniku
    /*
    printf("Preferred configuration (index %d):\n", preferred_index);
    for (int i = 0; i < num_joints; i++) {
        printf("%.4f ", preferred_config[i]);
    }
    printf("\n");
    */
                     
}

void InverseKinematics_Modified(const float a[], const float d[], const float T_interp[4][4][7][10], int index, 
                         const float q_actual[7], float q_out[7]) {

    float T[4][4][7];
    matrixExtractionT_interp(T_interp, index, T);
    

    // Ekstrakcja macierzy transformacji T04
    float T04[4][4];
    matrixExtractionT(T, 3, T04);

    // Kinematyka przestrzenna - obliczanie q1, q2, q3

    // q1

    // Punkt P04 1x3 (Punkt w ukladzie 4 - q4) 
    float P04_3[3] = {T04[0][3], T04[1][3], T04[2][3]};

    // Wyznaczenie q1_A i q1_B
    float alpha1 = atan2(P04_3[1], P04_3[0]);

    float sin_beta1 = d[2] / sqrt(P04_3[0] * P04_3[0] + P04_3[1] * P04_3[1]);
    float cos_beta1 = sqrt(1 - sin_beta1 * sin_beta1);

    float beta1_positive = atan2(sin_beta1, cos_beta1);
    float beta1_negative = atan2(sin_beta1, -cos_beta1);

    float q1_A = alpha1 - beta1_positive;
    float q1_B = alpha1 - beta1_negative;

    if (fabs(P04_3[0]) < 1e-6 && fabs(P04_3[1]) < 1e-6) {
        // Punkt docelowy leży na osi, kąt q1 jest nieokreślony
        q1_A = 0;
        q1_B = 0;
    }

    // q3

    // Wzor: P14 = ((T01(q1))^-1 * P04  

    // Zmiana P04 1x3 na homogeniczne współrzędne, dodanie jedynki jako czwartego elementu 1x4
    float P04[4] = {P04_3[0], P04_3[1], P04_3[2], 1.0};

    // Transformacje dla q1_A i q1_B
    float T01_q1_A[4][4] = {
        {cos(q1_A), -sin(q1_A), 0, 0},
        {sin(q1_A),  cos(q1_A), 0, 0},
        {0,                0,   1, d[0]},
        {0,                0,   0, 1}
    };

    float T01_q1_B[4][4] = {
        {cos(q1_B), -sin(q1_B), 0, 0},
        {sin(q1_B),  cos(q1_B), 0, 0},
        {0,                0,   1, d[0]},
        {0,                0,   0, 1}
    };

    
    // Wyznaczenie P14_q1_A

    // Potrzebne P01 oraz R01 z q1_A
    float P01_q1_A[3];
    vectorExtractionT(T01_q1_A, P01_q1_A);

    float R01_q1_A[3][3];
    matrixExtractionR(T01_q1_A, R01_q1_A);

    // Transpozycja macierzy R01_q1_A
    float R01_q1_A_T[3][3];
    transposeMatrix_3x3(R01_q1_A, R01_q1_A_T);

    float P01_q1_A_inv[3];
    multiplyMatrixVector_3x1(R01_q1_A_T, P01_q1_A, P01_q1_A_inv);

    float T01_q1_A_inv[4][4] = {
        {R01_q1_A_T[0][0], R01_q1_A_T[0][1], R01_q1_A_T[0][2], - P01_q1_A_inv[0]},
        {R01_q1_A_T[1][0], R01_q1_A_T[1][1], R01_q1_A_T[1][2], - P01_q1_A_inv[1]},
        {R01_q1_A_T[2][0], R01_q1_A_T[2][1], R01_q1_A_T[2][2], - P01_q1_A_inv[2]},
        {0,                               0,                0, 1}
    };

    float P14_q1_A[4];
    multiplyMatrixVector_4x1(T01_q1_A_inv, P04, P14_q1_A);


    // Wyznaczenie P14_q1_B

    // Potrzebne P01 oraz R01 z q1_B
    float P01_q1_B[3];
    vectorExtractionT(T01_q1_B, P01_q1_B);

    float R01_q1_B[3][3];
    matrixExtractionR(T01_q1_B, R01_q1_B);

    // Transpozycja macierzy R01_q1_B
    float R01_q1_B_T[3][3];
    transposeMatrix_3x3(R01_q1_B, R01_q1_B_T);

    float P01_q1_B_inv[3];
    multiplyMatrixVector_3x1(R01_q1_B_T, P01_q1_B, P01_q1_B_inv);

    float T01_q1_B_inv[4][4] = {
        {R01_q1_B_T[0][0], R01_q1_B_T[0][1], R01_q1_B_T[0][2], - P01_q1_B_inv[0]},
        {R01_q1_B_T[1][0], R01_q1_B_T[1][1], R01_q1_B_T[1][2], - P01_q1_B_inv[1]},
        {R01_q1_B_T[2][0], R01_q1_B_T[2][1], R01_q1_B_T[2][2], - P01_q1_B_inv[2]},
        {0,                               0,                0, 1}
    };

    float P14_q1_B[4];
    multiplyMatrixVector_4x1(T01_q1_B_inv, P04, P14_q1_B);



    // Wartości geometryczne dla P14
    float x_A = P14_q1_A[0];
    float z_A = P14_q1_A[2];

    float x_B = P14_q1_B[0];
    float z_B = P14_q1_B[2];

    // Standardowy przypadek dla q1_A
    float cos_beta_q1_A = (x_A * x_A + z_A * z_A - a[2] * a[2] - d[3] * d[3]) / (2 * a[2] * d[3]);
    cos_beta_q1_A = fmaxf(-1, fminf(1, cos_beta_q1_A)); // Ograniczenie wartości do zakresu [-1, 1]

    float beta_q3_A = acos(cos_beta_q1_A); // beta > 0 dla q1_A
    float beta_q3_B = -acos(cos_beta_q1_A); // beta < 0 dla q1_A

    float q3_A = M_PI / 2 + beta_q3_A;
    float q3_B = M_PI / 2 + beta_q3_B;

    // Standardowy przypadek dla q1_B

    float cos_beta_q1_B = (x_B * x_B + z_B * z_B - a[2] * a[2] - d[3] * d[3]) / (2 * a[2] * d[3]);
    cos_beta_q1_B = fmaxf(-1, fminf(1, cos_beta_q1_B)); // Ograniczenie wartości do zakresu [-1, 1]
    
    float beta_q3_C = acos(cos_beta_q1_B); // beta > 0 dla q1_B
    float beta_q3_D = -acos(cos_beta_q1_B); // beta > 0 dla q1_B

    float q3_C = M_PI / 2 + beta_q3_C;
    float q3_D = M_PI / 2 + beta_q3_D;

    // q2

    // Obliczenie α dla q1_A
    float alpha2_A = atan2(P14_q1_A[2], P14_q1_A[0]);

    // Wyznaczenie cos(beta) dla q1_A
    float r_A = sqrt(P14_q1_A[0] * P14_q1_A[0] + P14_q1_A[2] * P14_q1_A[2]);
    float cos_beta_q2_A = (P14_q1_A[0] * P14_q1_A[0] + P14_q1_A[2] * P14_q1_A[2] + a[2] * a[2] - d[3] * d[3]) / (2 * a[2] * r_A);
    
    // Ograniczenie wartości do przedziału [-1, 1]
    cos_beta_q2_A = fmaxf(-1, fminf(1, cos_beta_q2_A));

    // Obliczenie beta dla q2_A
    float beta_q2_A = acos(cos_beta_q2_A);
    float beta_q2_B = acos(cos_beta_q2_A);

    // Wyznaczenie q2 dla q1_A - Warunek sprawdzajacy znak beta (roznorodnosc rozwiazan)
    float q2_A, q2_B;

    if (beta_q3_B < 0){
        q2_A = - (alpha2_A + beta_q2_A); // Negacja kąta, bo jest on obliczany w złym kierunku
    } else if (beta_q3_B == 0){
        q2_A = - (alpha2_A + beta_q2_A);
    }
    else{
        q2_A = 0;
    }

   
    if (beta_q3_A > 0){
        q2_B = - (alpha2_A - beta_q2_B); // Negacja kąta, bo jest on obliczany w złym kierunku
    } else if (beta_q3_A == 0)
        q2_B = - (alpha2_A - beta_q2_B); 
    else{
        q2_B = 0;
    }

    // Obliczenie α dla q1_B
    float alpha2_B = atan2(P14_q1_B[2], P14_q1_B[0]); 

    // Wyznaczenie cos(beta) dla q1_B
    float r_B = sqrt(P14_q1_B[2] * P14_q1_B[2]  + P14_q1_B[0] * P14_q1_B[0]);
    float cos_beta_q2_B = ((P14_q1_B[0] * P14_q1_B[0]) + (P14_q1_B[2] * P14_q1_B[2] + a[2] * a[2] - d[3] * d[3])) / (2 * a[2] * r_B);

    // Ograniczenie wartości do przedziału [-1, 1]
    cos_beta_q2_B = fmaxf(-1, fminf(1, cos_beta_q2_B));

    // Obliczenie beta dla q1_B
    float beta_q2_C = acos(cos_beta_q2_B);
    float beta_q2_D = acos(cos_beta_q2_B);

    // Wyznaczenie q2 dla q1_B - Warunek sprawdzajacy znak beta (roznorodnosc rozwiazan)
    float q2_C, q2_D;

    if (beta_q3_D < 0){
        q2_C = - (alpha2_B + beta_q2_C); // Negacja kąta, bo jest on obliczany w złym kierunku
    } else if (beta_q3_D == 0){
        q2_C = - (alpha2_B + beta_q2_C);
    }
    else{
        q2_C = 0;
    }

   
    if (beta_q3_C > 0){
        q2_D = - (alpha2_B - beta_q2_D); // Negacja kąta, bo jest on obliczany w złym kierunku
    } else if (beta_q3_C == 0)
        q2_D = - (alpha2_B - beta_q2_D); 
    else{
        q2_D = 0;
    }

    // ---------------------------------------------------------------------------

    // Kinematyka orientacyjna - obliczanie q4, q5, q6

    // Ekstrakcja macierzy transformacji T0k
    float T0k[4][4];
    matrixExtractionT(T, 6, T0k);

    // Ekstrakcja macierzy transformacji T03
    float T03[4][4];
    matrixExtractionT(T, 2, T03);

    // R03 * R3k = R0k => R3k = (RO3)^-1 * R0k => R3k = (R03)^T * R0k
    float R0k[3][3];
    matrixExtractionR(T0k, R0k);

    // Wyznaczanie 4 roznych brakujacych macierzy R03
   
    float R03_A[3][3];
    Calculate_R03_x(q1_A, q2_A, q3_A, R03_A);

    float R03_B[3][3];
    Calculate_R03_x(q1_A, q2_B, q3_B, R03_B);

    float R03_C[3][3];
    Calculate_R03_x(q1_B, q2_C, q3_C, R03_C);

    float R03_D[3][3];
    Calculate_R03_x(q1_B, q2_D, q3_D, R03_D);
    
    // Wyznaczenie R3k

    float R03_A_T[3][3];
    transposeMatrix_3x3(R03_A, R03_A_T); // (R03_A)^T

    float R03_B_T[3][3];
    transposeMatrix_3x3(R03_B, R03_B_T);

    float R03_C_T[3][3];
    transposeMatrix_3x3(R03_C, R03_C_T);

    float R03_D_T[3][3];
    transposeMatrix_3x3(R03_D, R03_D_T);

    float R3k_A[3][3];
    multiplyMatrix_3x3(R03_A_T, R0k, R3k_A); // (R03_A_T) * R0k = R3k_A 

    float R3k_B[3][3];
    multiplyMatrix_3x3(R03_B_T, R0k, R3k_B);

    float R3k_C[3][3];
    multiplyMatrix_3x3(R03_C_T, R0k, R3k_C);

    float R3k_D[3][3];
    multiplyMatrix_3x3(R03_D_T, R0k, R3k_D);

    // q5 (8 rozwiazan)

    // Zmienna sq5 i cq5 dla różnych konfiguracji
    float sq5_A, cq5_A, q5_A, q5_B;
    float sq5_B, cq5_B, q5_C, q5_D;
    float sq5_C, cq5_C, q5_E, q5_F;
    float sq5_D, cq5_D, q5_G, q5_H;

    // z R03_A
    sq5_A = sqrt((R3k_A[0][2] * R3k_A[0][2]) + (R3k_A[2][2] * R3k_A[2][2]));
    cq5_A = -R3k_A[1][2];

    q5_A = atan2(sq5_A, cq5_A);
    q5_B = -atan2(sq5_A, cq5_A);

    // z R03_B
    sq5_B = sqrt((R3k_B[0][2] * R3k_B[0][2]) + (R3k_B[2][2] * R3k_B[2][2]));
    cq5_B = -R3k_B[1][2];

    q5_C = atan2(sq5_B, cq5_B);
    q5_D = -atan2(sq5_B, cq5_B);

    // z R03_C
    sq5_C = sqrt((R3k_C[0][2] * R3k_C[0][2]) + (R3k_C[2][2] * R3k_C[2][2]));
    cq5_C = -R3k_C[1][2];

    q5_E = atan2(sq5_C, cq5_C);
    q5_F = -atan2(sq5_C, cq5_C);

    // z R03_D
    sq5_D = sqrt((R3k_D[0][2] * R3k_D[0][2]) + (R3k_D[2][2] * R3k_D[2][2]));
    cq5_D = -R3k_D[1][2];

    q5_G = atan2(sq5_D, cq5_D);
    q5_H = -atan2(sq5_D, cq5_D);

    // q4 (8 rozwiazan)

    // Zmienne dla q4
    float q4_A, q4_B, q4_C, q4_D, q4_E, q4_F, q4_G, q4_H;

    // q4 dla R03_A
    q4_A = atan2(R3k_A[2][2], R3k_A[0][2]);
    q4_B = atan2(-R3k_A[2][2], -R3k_A[0][2]);

    // q4 dla R03_B
    q4_C = atan2(R3k_B[2][2], R3k_B[0][2]);
    q4_D = atan2(-R3k_B[2][2], -R3k_B[0][2]);

    // q4 dla R03_C
    q4_E = atan2(R3k_C[2][2], R3k_C[0][2]);
    q4_F = atan2(-R3k_C[2][2], -R3k_C[0][2]);

    // q4 dla R03_D
    q4_G = atan2(R3k_D[2][2], R3k_D[0][2]);
    q4_H = atan2(-R3k_D[2][2], -R3k_D[0][2]);

    // q6 (8 rozwiazan)

    // Zmienne dla q6
    float q6_A, q6_B, q6_C, q6_D, q6_E, q6_F, q6_G, q6_H;

    // q6 dla R03_A
    q6_A = atan2(R3k_A[1][1], R3k_A[1][0]);
    q6_B = atan2(-R3k_A[1][1], -R3k_A[1][0]);

    // q6 dla R03_B
    q6_C = atan2(R3k_B[1][1], R3k_B[1][0]);
    q6_D = atan2(-R3k_B[1][1], -R3k_B[1][0]);

    // q6 dla R03_C
    q6_E = atan2(R3k_C[1][1], R3k_C[1][0]);
    q6_F = atan2(-R3k_C[1][1], -R3k_C[1][0]);

    // q6 dla R03_D
    q6_G = atan2(R3k_D[1][1], R3k_D[1][0]);
    q6_H = atan2(-R3k_D[1][1], -R3k_D[1][0]);

    // Konfiguracje
    float configs[8][7] = {
        {q1_A, q2_A, q3_A, q4_A, q5_A, q6_A, 0},
        {q1_A, q2_A, q3_A, q4_B, q5_B, q6_B, 0},
        {q1_A, q2_B, q3_B, q4_C, q5_C, q6_C, 0},
        {q1_A, q2_B, q3_B, q4_D, q5_D, q6_D, 0},
        {q1_B, q2_C, q3_C, q4_E, q5_E, q6_E, 0},
        {q1_B, q2_C, q3_C, q4_F, q5_F, q6_F, 0},
        {q1_B, q2_D, q3_D, q4_G, q5_G, q6_G, 0},
        {q1_B, q2_D, q3_D, q4_H, q5_H, q6_H, 0}
    };


    //printConfigsRad(configs, 8, 7);
    //printConfigsDeg(configs, 8, 7);

    // Inicjalizacja minimalnej różnicy i preferowanej konfiguracji
    int num_configs = 8;
    int num_joints = 7;
    float min_diff = FLT_MAX;
    int preferred_index = -1; // Numer wybranej konfiguracji
    float preferred_config[7] = {0}; // Wybrana konfiguracja

    // Kryterium tolerancji
    for (int i = 0; i < num_configs; i++) {
        float diff = 0.0;

        // Obliczanie różnicy (norma euklidesowa)
        for (int j = 0; j < num_joints; j++) {
            diff += pow(configs[i][j] - q_actual[j], 2);
        }
        diff = sqrt(diff);

        // Aktualizacja minimalnej różnicy i preferowanej konfiguracji
        if (diff < min_diff) {
            min_diff = diff;
            preferred_index = i;

            // Kopiowanie konfiguracji
            for (int j = 0; j < num_joints; j++) {
                preferred_config[j] = configs[i][j];
            }
        }
    }

    for (int i = 0; i < num_joints; i++) {
        q_out[i] = preferred_config[i];
    }

    // Wyświetlenie wyniku
    /*
    printf("Preferred configuration (index %d):\n", preferred_index);
    for (int i = 0; i < num_joints; i++) {
        printf("%.4f ", preferred_config[i]);
    }
    printf("\n");
    */
                     
}

// Funkcja konwersji macierzy rotacji na kwaternion
void rotationMatrixToQuaternion(const float R[3][3], float q[4]) {

    float qw = sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2;
    float qx = (R[2][1] - R[1][2]) / (4 * qw);
    float qy = (R[0][2] - R[2][0]) / (4 * qw);
    float qz = (R[1][0] - R[0][1]) / (4 * qw);
    q[0] = qw;
    q[1] = qx;
    q[2] = qy;
    q[3] = qz;
}

void quaternionToRotationMatrix(const float q[4], float R[3][3]) {
    // Ekstrakcja składowych kwaternionu
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    // Obliczenie elementów macierzy rotacji
    R[0][0] = 1 - 2 * qy * qy - 2 * qz * qz;
    R[0][1] = 2 * qx * qy - 2 * qw * qz;
    R[0][2] = 2 * qx * qz + 2 * qw * qy;

    R[1][0] = 2 * qx * qy + 2 * qw * qz;
    R[1][1] = 1 - 2 * qx * qx - 2 * qz * qz;
    R[1][2] = 2 * qy * qz - 2 * qw * qx;

    R[2][0] = 2 * qx * qz - 2 * qw * qy;
    R[2][1] = 2 * qy * qz + 2 * qw * qx;
    R[2][2] = 1 - 2 * qx * qx - 2 * qy * qy;
}

// Funkcja interpolacji kwaternionów (SLERP)
void slerp(float q1[4], const float q2[4], float t, float q_interp[4]) {

    float dot_product = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    if (dot_product < 0.0f) {
        dot_product = -dot_product;
        for (int i = 0; i < 4; i++) {
            q1[i] = -q1[i];
        }
    }

    const float EPSILON = 1e-6;
    if (dot_product > 1.0f - EPSILON) {
        for (int i = 0; i < 4; i++) {
            q_interp[i] = (1 - t) * q1[i] + t * q2[i];
        }

        float norm = sqrt(q_interp[0] * q_interp[0] + q_interp[1] * q_interp[1] +
                           q_interp[2] * q_interp[2] + q_interp[3] * q_interp[3]);

        for (int i = 0; i < 4; i++) {
            q_interp[i] /= norm;
        }
        return;
    }

    float theta_0 = acos(dot_product);
    float theta = theta_0 * t;
    
    float q3[4];
    
    for (int i = 0; i < 4; i++) {
        q3[i] = q2[i] - q1[i] * dot_product;
    }

    float norm2 = sqrt(q3[0] * q3[0] + q3[1] * q3[1] +
                           q3[2] * q3[2] + q3[3] * q3[3]);

    for (int i = 0; i < 4; i++) {
        q3[i] = q3[i]/norm2;
    }

    for (int i = 0; i < 4; i++) {
        q_interp[i] = q1[i] * cos(theta) + q3[i] * sin(theta);
    }
}


// Funkcja interpolacji pojedynczego punktu
void interpolatePoint(const float T_P1[4][4][7], const float T_P2[4][4][7], float t, int num_matrices, float T_interp[4][4][7]) {
    for (int i = 0; i < num_matrices; i++) {
        
        float T_P1_1[4][4];
        matrixExtractionT(T_P1, i, T_P1_1);
        float R_P1[3][3];
        matrixExtractionR(T_P1_1, R_P1);
        float P_P1[3];
        vectorExtractionT(T_P1_1, P_P1);

        float T_P2_1[4][4];
        matrixExtractionT(T_P2, i, T_P2_1);
        float R_P2[3][3];
        matrixExtractionR(T_P2_1, R_P2);
        float P_P2[3];
        vectorExtractionT(T_P2_1, P_P2);

        float q_P1[4], q_P2[4];
        rotationMatrixToQuaternion(R_P1, q_P1);
        rotationMatrixToQuaternion(R_P2, q_P2);

        float q_interp[4];
        slerp(q_P1, q_P2, t, q_interp);

        float R_interp[3][3];
        quaternionToRotationMatrix(q_interp, R_interp);

        float P_interp[3];
        for (int i = 0; i < 3; i++){
            P_interp[i] = (1 - t) * P_P1[i] + t * P_P2[i];
        }

        
            // Zapisanie macierzy rotacji (1:3, 1:3)
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    T_interp[row][col][i] = R_interp[row][col];
                }
            }

            // Zapisanie wektora pozycji (1:3, 4)
            for (int row = 0; row < 3; row++) {
                T_interp[row][3][i] = P_interp[row];
            }

            // Zapisanie jednorodnej reprezentacji (4, :, i)
            T_interp[3][0][i] = 0.0;
            T_interp[3][1][i] = 0.0;
            T_interp[3][2][i] = 0.0;
            T_interp[3][3][i] = 1.0;

            
            
        
    }
}



// Główna funkcja do interpolacji SLERP
void SLERP_interpolation(const float T_P1[4][4][7], const float T_P2[4][4][7], int points_num, float T_interp[4][4][7][10]) {
    for (int i = 0; i < points_num; i++) {
        float t = (float)(i + 1) / (points_num + 1);
        float T_interp_mini [4][4][7] = {0}; 
        matrixExtractionT_interp(T_interp, i, T_interp_mini);
        interpolatePoint(T_P1, T_P2, t, 7, T_interp_mini);

        // Przypisanie wartości z T_interp_mini do T_interp dla i-tego wymiaru
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                for (int layer = 0; layer < 7; layer++) {
                    T_interp[row][col][layer][i] = T_interp_mini[row][col][layer];
                }
            }
        }

        float T_test[4][4][7];
        matrixExtractionT_interp(T_interp, i, T_test);
        //printAllMatrixT_4x4x7(T_test);
    }
}

