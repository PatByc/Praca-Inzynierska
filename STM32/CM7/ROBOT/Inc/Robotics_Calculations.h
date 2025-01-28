#include <math.h>
#include <stdio.h> 
#include <float.h>
#include <stddef.h>
#include <string.h>

#define M_PI 3.14159265358979323846

// Deklaracje funkcji
void ToEulers(float T0k[4][4], float Eulers[6]);

void ForwardKinematics(const float alpha[], const float a[], const float q[], const float d[], 
                         float Eulers[], float T[4][4][7]);

void multiplyMatrix_4x4(const float A[4][4], const float B[4][4], float C[4][4]);
void multiplyMatrix_3x3(const float A[3][3], const float B[3][3], float C[3][3]);

void matrixExtractionT(const float Txxi[4][4][7], int index, float Txx[4][4]);
void matrixExtractionT_interp(const float T_interp[4][4][7][10], int index, float T_P2[4][4][7]);
void matrixExtractionR(const float T[4][4], float R[3][3]);


void vectorExtractionT(const float T[4][4], float P[3]);
void transposeMatrix_3x3(const float matrix[3][3], float result[3][3]);
void multiplyMatrixVector_3x1(float matrix[3][3], float vector[3], float result[3]);
void multiplyMatrixVector_4x1(float matrix[4][4], float vector[4], float result[4]);
void Calculate_R03_x(float q1_x, float q2_x, float q3_x, float R03_x[3][3]);
void toleranceMatrix(float matrix[3][3]);


void printVector(const float vector[], int size, const char* name);
void printMatrixT_4x4(const float matrix[4][4], const char* name);
void printMatrix_3x3(const float matrix[3][3], const char* name);
void printMatrixT_4x4x7(const float matrix[4][4][7], int num_matrices); 
void printAllMatrixT_4x4x7(const float matrix[4][4][7]);
void printConfigsRad(const float configs[8][7], int rows, int cols);

void slerp(float q1[4], const float q2[4], float t, float q_interp[4]);
void rotationMatrixToQuaternion(const float R[3][3], float q[4]);
void interpolatePoint(const float T_P1[4][4][7], const float T_P2[4][4][7], float t, int num_matrices, float T_interp[4][4][7]);
void SLERP_interpolation(const float T_P1[4][4][7], const float T_P2[4][4][7], int points_num, float T_interp[4][4][7][10]);
void quaternionToRotationMatrix(const float q[4], float R[3][3]);

void InverseKinematics(const float a[], const float d[], const float T[4][4][7], 
                         const float q_actual[7], float q_out[7]);
void InverseKinematics_Modified(const float a[], const float d[], const float T_interp[4][4][7][10], int index, 
                         const float q_actual[7], float q_out[7]);
void MakePath(float a[7], float d[7], float q_P1[7], float T_P1[4][4][7], float q_Px[7], float T_Px[4][4][7], int num_points, float q_path_results[][7], const char* path_name);
void SavePath(float q_results[10][7], float q_P1[7], float q_P2[7], float q_P3[7],
                float q_P4[7], float q_P5[7], float q_P6[7], float q_P7[7], float q_P8[7], float q_P9[7], float q_P10[7]);
void PrintPath(float q_results[10][7], const char* path_name);