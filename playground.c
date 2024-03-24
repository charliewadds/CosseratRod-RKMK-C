//
// Created by Charlie Wadds on 2024-03-19.
//

#include <stdio.h>
#include <arm_neon.h>
#include <stdio.h>
#include <time.h>


void matrix_multiply_4x4(float32_t *A, float32_t *B, float32_t *C) {
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            C[i*4+j] = 0;
            for(int k = 0; k < 4; k++){
                C[i*4+j] += A[i*4+k] * B[k*4+j];
            }
        }
    }
}


void matrix_multiply_4x4_neon(float32_t *A, float32_t *B, float32_t *C) {
    // these are the columns A
    float32x4_t A0;
    float32x4_t A1;
    float32x4_t A2;
    float32x4_t A3;

    // these are the columns B
    float32x4_t B0;
    float32x4_t B1;
    float32x4_t B2;
    float32x4_t B3;

    // these are the columns C
    float32x4_t C0;
    float32x4_t C1;
    float32x4_t C2;
    float32x4_t C3;

    A0 = vld1q_f32(A);
    A1 = vld1q_f32(A+4);
    A2 = vld1q_f32(A+8);
    A3 = vld1q_f32(A+12);

    // Zero accumulators for C values
    C0 = vmovq_n_f32(0);
    C1 = vmovq_n_f32(0);
    C2 = vmovq_n_f32(0);
    C3 = vmovq_n_f32(0);

    // Multiply accumulate in 4x1 blocks, i.e. each column in C
    B0 = vld1q_f32(B);
    C0 = vfmaq_laneq_f32(C0, A0, B0, 0);
    C0 = vfmaq_laneq_f32(C0, A1, B0, 1);
    C0 = vfmaq_laneq_f32(C0, A2, B0, 2);
    C0 = vfmaq_laneq_f32(C0, A3, B0, 3);
    vst1q_f32(C, C0);

    B1 = vld1q_f32(B+4);
    C1 = vfmaq_laneq_f32(C1, A0, B1, 0);
    C1 = vfmaq_laneq_f32(C1, A1, B1, 1);
    C1 = vfmaq_laneq_f32(C1, A2, B1, 2);
    C1 = vfmaq_laneq_f32(C1, A3, B1, 3);
    vst1q_f32(C+4, C1);

    B2 = vld1q_f32(B+8);
    C2 = vfmaq_laneq_f32(C2, A0, B2, 0);
    C2 = vfmaq_laneq_f32(C2, A1, B2, 1);
    C2 = vfmaq_laneq_f32(C2, A2, B2, 2);
    C2 = vfmaq_laneq_f32(C2, A3, B2, 3);
    vst1q_f32(C+8, C2);

    B3 = vld1q_f32(B+12);
    C3 = vfmaq_laneq_f32(C3, A0, B3, 0);
    C3 = vfmaq_laneq_f32(C3, A1, B3, 1);
    C3 = vfmaq_laneq_f32(C3, A2, B3, 2);
    C3 = vfmaq_laneq_f32(C3, A3, B3, 3);
    vst1q_f32(C+12, C3);
}

void printMatrixList(float32_t *A){
    for(int i = 0; i < 16; i++){
        printf("%f ", A[i]);
        if(i % 4 == 3){
            printf("\n");
            }
    }
}
int main(void){
    float32_t A[16] = {1, 0, 0, 0,
                       0, 0, -1, -0.65,
                       0, 1, 0, 0.5,
                       0, 0, 0, 1};
    float32_t B[16] = {1, 0, 0, 0,
                       0, 1, 0, 0,
                       0, 0, 1, 0.25,
                       0, 0, 0, 1};
    float32_t C[16] = {0};
    float32_t D[16] = {0};

    clock_t start_neon, end_neon, start_reg, end_reg;
    double time_taken_neon, time_taken_reg;



    // Measure time for matrix_multiply_4x4
    start_reg = clock();
    matrix_multiply_4x4(A, B, D);
    end_reg = clock();
    time_taken_reg = ((double)(end_reg - start_reg)) / CLOCKS_PER_SEC;
    printf("Time taken by regular function: %f seconds\n", time_taken_reg);
    // Measure time for matrix_multiply_4x4_neon
    start_neon = clock();
    matrix_multiply_4x4_neon(A, B, C);
    end_neon = clock();
    time_taken_neon = ((double)(end_neon - start_neon)) / CLOCKS_PER_SEC;
    printf("Time taken by NEON function: %f seconds\n", time_taken_neon);

    // Print the resulting matrices
    printf("Matrix C (NEON):\n");
    printMatrixList(C);

    printf("Matrix D (Regular):\n");
    printMatrixList(D);

    return 0;
}

