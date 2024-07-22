#include <assert.h>
#include <math.h>
#include <printf.h>
#include "Matrices.h"
#include <string.h>
#include <gsl/gsl_linalg.h>
#include <arm_neon.h>
#include <simd/simd.h>





gsl_matrix *matrix_to_gsl(matrix *matrix, gsl_matrix *out){
    //gsl_matrix *m = gsl_matrix_alloc(matrix->numRows, matrix->numCols);
    for(int i = 0; i < matrix->numRows; i++){
        for(int j = 0; j < matrix->numCols; j++){
            gsl_matrix_set(out, i, j, matrix->data[(i * matrix->numCols) + j]);
        }
    }
    return out;
}

gsl_matrix *copy_matrix_to_gsl(matrix *matrix, gsl_matrix *out){
    //gsl_matrix *m = gsl_matrix_alloc(matrix->numRows, matrix->numCols);
    for(int i = 0; i < matrix->numRows; i++){
        for(int j = 0; j < matrix->numCols; j++){
            gsl_matrix_set(out, i, j, matrix->data[(i * matrix->numCols) + j]);
        }
    }
    return out;
}

matrix *expm(matrix *A, matrix *result){
    assert(A->square == 1);

    gsl_matrix *gsl_A = gsl_matrix_alloc(A->numRows, A->numCols);
    gsl_matrix *gsl_result = gsl_matrix_alloc(A->numRows, A->numCols);
    copy_matrix_to_gsl(A, gsl_A);
    //matrix_to_gsl(A, gsl_A);
    gsl_linalg_exponential_ss(gsl_A, gsl_result, GSL_PREC_DOUBLE);

    gsl_to_matrix(gsl_result, result);
    gsl_matrix_free(gsl_A);
    gsl_matrix_free(gsl_result);
    return result;
}

matrix *gsl_to_matrix(gsl_matrix *gsl_matrix, matrix *result){
    //matrix *m = matrix_new(gsl_matrix->size1, gsl_matrix->size2);
    //zeroMatrix(result);
    assert(gsl_matrix->size1* gsl_matrix->size2 == result->numRows * result->numCols);
    for(int i = 0; i < result->numRows; i++){
        for(int j = 0; j < result->numCols; j++){
            result->data[(i * result->numCols) + j] = gsl_matrix_get(gsl_matrix, i, j);
        }
    }
    return result;
}
//make sure this is freed!!!
matrix *matrix_new(int num_rows, int num_cols) {
    assert(num_rows != 0);
    assert(num_cols != 0);

    //check for malloc fail

    matrix *m = malloc(sizeof(matrix));

    if(m == NULL){
        printf("malloc failed");
        assert(0);
        return NULL;
    }

    m->numRows = num_rows;
    m->numCols = num_cols;
    m->data = calloc((num_rows*num_cols), sizeof(double));
    if(num_rows == num_cols) {
        m->square = 1;
    }
    return m;
}



void matrix_free(matrix *m) {
    assert(m != NULL);

    free(m->data);
    free(m);
}


#ifndef NEON
matrix *matrix_add(matrix *m1, matrix *m2, matrix *result){
    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    //matrix *result = matrix_new(m1->numRows, m1->numCols);

    assert(result->numRows == m1->numRows);
    assert(result->numCols == m1->numCols);

    matrix *temp;
    if(m1 == result || m2 == result){
        temp = matrix_new(result->numRows, result->numCols);
        copyMatrix(m1, temp);
    }else{
        temp = result;
        //zeroMatrix(temp);
    }

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            temp->data[(i * temp->numCols) + j] = m1->data[(i * m1->numCols) + j] + m2->data[(i * m2->numCols) + j];
        }
    }

    if(m1 == result || m2 == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;

}
#endif


#ifdef NEON


matrix *matrix_add(matrix *m1, matrix *m2, matrix *result) {
    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    assert(result->numRows == m1->numRows);
    assert(result->numCols == m1->numCols);

    matrix *temp;
    if (m1 == result || m2 == result) {
        temp = matrix_new(result->numRows, result->numCols);
        copyMatrix(m1, temp);
    } else {
        temp = result;
    }

    int num_elements = m1->numRows * m1->numCols;
    int i = 0;

    // Use NEON intrinsics for SIMD processing
    for (; i <= num_elements - 2; i += 2) {
        float64x2_t m1_vec = vld1q_f64(&m1->data[i]);
        float64x2_t m2_vec = vld1q_f64(&m2->data[i]);
        float64x2_t result_vec = vaddq_f64(m1_vec, m2_vec);
        vst1q_f64(&temp->data[i], result_vec);
    }

    // Process remaining elements
    for (; i < num_elements; i++) {
        temp->data[i] = m1->data[i] + m2->data[i];
    }

    if (m1 == result || m2 == result) {
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;
}

#endif
matrix *matrix_add3(matrix *m1,matrix *m2, matrix *m3, matrix *result){

    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    assert(m1->numRows == m3->numRows);
    assert(m1->numCols == m3->numCols);

    assert(result->numRows == m1->numRows);
    assert(result->numCols == m1->numCols);


    matrix *temp = matrix_new(m1->numRows, m1->numCols);
    matrix_add(m1, m2, temp);
    matrix_add(temp, m3, result);

    matrix_free(temp);
    return result;
}

#ifndef NEON

matrix *matrix_scalar_mul(matrix *m, double scalar, matrix *result){
    //matrix *result = matrix_new(m->numRows, m->numCols);

    assert(result->numRows == m->numRows);
    assert(result->numCols == m->numCols);

    matrix *temp;
    if(m == result){
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;

    }
    zeroMatrix(temp);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            temp->data[(i * temp->numCols) + j] = m->data[(i * m->numCols) + j] * scalar;
        }
    }
    if(m == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;
}
#endif

//use neon
#ifdef NEON
matrix *matrix_scalar_mul(matrix *m, double scalar, matrix *result){
assert(result->numRows == m->numRows);
    assert(result->numCols == m->numCols);

    matrix *temp;
    if(m == result){
        temp = matrix_new(result->numRows, result->numCols);
        copyMatrix(m, temp);
    }else{
        temp = result;
    }

    int num_elements = m->numRows * m->numCols;
    int i = 0;

    // Use NEON intrinsics for SIMD processing
    for (; i <= num_elements - 2; i += 2) {
        float64x2_t m_vec = vld1q_f64(&m->data[i]);
        float64x2_t scalar_vec = vdupq_n_f64(scalar);
        float64x2_t result_vec = vmulq_f64(m_vec, scalar_vec);
        vst1q_f64(&temp->data[i], result_vec);
    }

    // Process remaining elements
    for (; i < num_elements; i++) {
        temp->data[i] = m->data[i] * scalar;
    }

    if (m == result) {
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;


}
#endif
matrix *matrix_scalar_mul_chain(matrix *m, double scalar){
    matrix *result = matrix_new(m->numRows, m->numCols);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[(i * result->numCols) + j] = m->data[(i * m->numCols) + j] * scalar;
        }
    }
    matrix_free(m);
    return result;
}


//todo make sure to free the result
int *matrix_shape(matrix *m){
    int *shape = malloc(sizeof(int) * 2);

    // if (shape == NULL) {
    //     // Handle memory allocation failure
    //     return NULL;
    // }

    shape[0] = m->numCols;
    shape[1] = m->numRows;
    free(shape);
    return shape;
}




matrix *matrix_solve(matrix *A, matrix *b, matrix *result){
    assert(A->numRows == b->numRows);
    assert(A->numCols == A->numRows);


    assert(Det(A) != 0);
    //assert(Det(b) != 0);

    matrix *A_inv = matrix_new(A->numRows, A->numCols);
    matrix_inverse(A, A_inv);
    matMult(A_inv, b, result);
    matrix_free(A_inv);

    return result;
}

matrix *matrix_inverse(matrix *m, matrix *result){
    assert(m->square == 1);

    assert(result->numRows == m->numRows);
    assert(result->numCols == m->numCols);

    //assert(Det(m) != 0);
    gsl_matrix *gsl_m = gsl_matrix_alloc(m->numRows, m->numCols);
    matrix_to_gsl(m, gsl_m);
    gsl_permutation *p = gsl_permutation_alloc(m->numRows);

    int signum;
    gsl_linalg_LU_decomp(gsl_m, p, &signum);
    gsl_linalg_LU_invx(gsl_m, p);
    gsl_to_matrix(gsl_m, result);
    gsl_matrix_free(gsl_m);
    gsl_permutation_free(p);

    return result;
}


//todo does this need to be dynamically allocated?
double dot(matrix *m1, matrix *m2){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == m2->numCols);
    double result = 0;
    //matrix *result = matrix_new(m1->numRows, m2->numCols);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            result += m1->data[(i * m1->numCols) + j] * m2->data[(i * m2->numCols) + j];
        }
    }

    return result;
}

matrix *cross(matrix *m1, matrix *m2, matrix *result){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == m2->numCols);

    //matrix *result = matrix_new(m1->numRows, m2->numCols);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            result->data[(i * result->numCols) + j] = m1->data[(i * m1->numCols) + j] * m2->data[(i * m2->numCols) + j];
        }
    }

    return result;
}

matrix *matrix_transpose(matrix *m, matrix *result){
    //matrix *result = matrix_new(m->numCols, m->numRows);
    assert(result->numRows == m->numCols);
    assert(result->numCols == m->numRows);


    matrix *temp;
    if(m == result){
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        //zeroMatrix(temp);
    }
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            temp->data[(j * temp->numCols) + i] = m->data[(i * m->numCols) + j];
        }
    }
    if(m == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }


    return result;
}

//transpose a 6x6 matrix using memory locations
matrix *matrix_transpose_6x6(matrix *m, matrix *result){
    //matrix *result = matrix_new(m->numCols, m->numRows);
    assert(result->numRows == 6);
    assert(result->numCols == 6);

    assert(m->numRows == 6);
    assert(m->numCols == 6);


    matrix *temp;
    if(m == result){
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        //zeroMatrix(temp);
    }


    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 6; j++){
            temp->data[(j * temp->numCols) + i] = m->data[(i * m->numCols) + j];
        }
    }
    if(m == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }


    return result;
}



matrix *eye(matrix *result){

    assert(result->numRows == result->numCols);

    memset(result->data, 0, result->numRows * result->numCols * sizeof(double));
    for(int i = 0; i < result->numRows; i++){
        result->data[(i * result->numCols) + i] = 1;
    }
    return result;
}


matrix *matrix_rand(int num_rows, int num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            m->data[(i * m->numCols) + j] = (double)rand() / RAND_MAX;
        }
    }
    return m;
}

double matrix_sumSelf(matrix *m){
    double result = 0;
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result += m->data[(i * m->numCols) + j];
        }
    }
    return result;
}
matrix *zeros(int num_rows, int num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    memset(m->data, 0, num_rows * num_cols * sizeof(double));
    return m;
}

//todo I am sure there is some bitwise wizardry that can be done here
matrix *zeroMatrix(matrix *m){

        if(m == NULL){
            return NULL;
        }
        memset(m->data, 0, m->numRows * m->numCols * sizeof(double));
        return m;

}

matrix *ones(int num_rows, int num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    memset(m->data, 1, num_rows * num_cols * sizeof(double));
    return m;
}


void setSection(matrix *m, int startRow, int endRow, int startCol, int endCol, matrix *section){
    assert(startRow <= endRow);
    assert(startCol <= endCol);
    assert(endRow <= m->numRows);
    assert(endCol <= m->numCols);


    for(int i = startRow; i <= endRow; i++){
        for(int j = startCol; j <= endCol; j++){
            m->data[(i * m->numCols) + j] = section->data[((i-startRow) * section->numCols) + (j-startCol)];
        }
    }


}


void copyMatrix(matrix *m, matrix *result){
    assert(m->numRows == result->numRows);
    assert(m->numCols == result->numCols);
    //zeroMatrix(result);
    memcpy(result->data, m->data, m->numRows * m->numCols * sizeof(double));
}
void getSetSection(matrix *get, matrix *set, int getStartRow, int getEndRow, int getStartCol, int getEndCol, int setStartRow, int setEndRow, int setStartCol, int setEndCol) {
    // Check if input parameters are within bounds
    assert(getStartRow <= getEndRow && getStartCol <= getEndCol);
    assert(getEndRow < get->numRows && getEndCol < get->numCols);
    assert(setStartRow <= setEndRow && setStartCol <= setEndCol);
    assert(setEndRow < set->numRows && setEndCol < set->numCols);


    for (int i = 0; i <= getEndRow - getStartRow; i++) {
        for (int j = 0; j <= getEndCol - getStartCol; j++) {

            // Copy data from the source to the destination matrix
            set->data[((setStartRow + i) * set->numCols) + (setStartCol + j)] = get->data[((getStartRow + i) * get->numCols) + (getStartCol + j)];
        }
    }
}

matrix *getSection(matrix *m, int startRow, int endRow, int startCol, int endCol, matrix *result){

    assert(startRow <= endRow);
    assert(startCol <= endCol);
    assert(endRow <= m->numRows);
    assert(endCol <= m->numCols);

    assert(result->numRows == (endRow - startRow)+1);
    assert(result->numCols == (endCol - startCol)+1);

    for(int i = startRow; i <= endRow; i++){
        for(int j = startCol; j <= endCol; j++){
            result->data[((i-startRow) * result->numCols) + (j- startCol)] = m->data[(i * m->numCols) + j];

        }
    }
    result->numRows = (endRow - startRow)+1;
    result->numCols = (endCol - startCol)+1;

    return result;
}

matrix *matrix_sin(matrix *m){
    matrix *result = matrix_new(m->numRows, m->numCols);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[(i * result->numCols) + j] =  sin(m->data[(i * m->numCols) + j]);
        }
    }
    return result;

}

matrix *matMult(matrix *m1, matrix *m2, matrix *result) {
    // Ensure the matrices have compatible dimensions for multiplication
    assert(m1->numCols == m2->numRows);

    // Ensure the result matrix has the correct dimensions
    assert((result->numRows == m1->numRows && result->numCols == m2->numCols) || (result->numRows == m1->numCols && result->numCols == m2->numRows));




    matrix *temp;

    if(m1 == result || m2 == result) {
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        zeroMatrix(temp);
    }

    for (int i = 0; i < m1->numRows; i++) {
        for (int j = 0; j < m2->numCols; j++) {
            for (int k = 0; k < m1->numCols; k++) {
                temp->data[(i * temp->numCols) + j] += m1->data[(i * m1->numCols) + k] * m2->data[(k * m2->numCols) + j];
            }
        }
    }

    if(m1 == result || m2 == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }



    return result;
}


#ifndef NEON
//multiply a 3x3 matrix by a 3x1 vector
matrix *matMult_3x3_3x1(matrix *m1, matrix *m2, matrix *result){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == 3);
    assert(m2->numCols == 1);
    assert(result->numRows == 3);
    assert(result->numCols == 1);

    matrix *temp;

    if(m1 == result || m2 == result) {
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        zeroMatrix(temp);
    }

    #pragma unroll
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 1; j++){
            for(int k = 0; k < 3; k++){
                temp->data[(i * temp->numCols) + j] += m1->data[(i * m1->numCols) + k] * m2->data[(k * m2->numCols) + j];
            }
        }
    }

    if(m1 == result || m2 == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;

}
#endif
#ifdef NEON
//multiply a 3x3 matrix by a 3x1 vector using apple simd_double3x3
matrix *matMult_3x3_3x1(matrix *m1, matrix *m2, matrix *result) {
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == 3);
    assert(m2->numCols == 1);
    assert(result->numRows == 3);
    assert(result->numCols == 1);

    simd_double3 m1_c1_simd = simd_make_double3(m1->data[0], m1->data[3], m1->data[6]);
    simd_double3 m1_c2_simd = simd_make_double3(m1->data[1], m1->data[2], m2->data[7]);
    simd_double3 m1_c3_simd = simd_make_double3(m1->data[2], m1->data[5], m1->data[8]);

    simd_double3x3 m1_simd = simd_matrix(m1_c1_simd, m1_c2_simd, m1_c3_simd);

    simd_double3 m2_simd = simd_make_double3(m2->data[0], m2->data[1], m2->data[2]);

    simd_double3 result_simd = simd_mul(m1_simd, m2_simd);


    for(int i = 0; i < 3; i++){
        result->data[i] = result_simd[i];
    }
    return result;

}
#endif

//multiply a 3x3 matrix by a 3x1 vector
matrix *matMult_6x6_6x1(matrix *m1, matrix *m2, matrix *result){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == 6);
    assert(m2->numCols == 1);
    assert(result->numRows == 6);
    assert(result->numCols == 1);

    matrix *temp;

    if(m1 == result || m2 == result) {
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        zeroMatrix(temp);
    }

#pragma unroll
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 1; j++){
            for(int k = 0; k < 6; k++){
                temp->data[(i * temp->numCols) + j] += m1->data[(i * m1->numCols) + k] * m2->data[(k * m2->numCols) + j];
            }
        }
    }

    if(m1 == result || m2 == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;

}

//multiply a 3x3 matrix by a 3x1 vector
matrix *matMult_4x4_4x4(matrix *m1, matrix *m2, matrix *result){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == 4);
    assert(m2->numCols == 4);
    assert(result->numRows == 4);
    assert(result->numCols == 4);

    matrix *temp;

    if(m1 == result || m2 == result) {
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        zeroMatrix(temp);
    }

#pragma unroll
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            for(int k = 0; k < 4; k++){
                temp->data[(i * temp->numCols) + j] += m1->data[(i * m1->numCols) + k] * m2->data[(k * m2->numCols) + j];
            }
        }
    }

    if(m1 == result || m2 == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;

}

//multiply a 3x3 matrix by a 3x1 vector
matrix *matMult_3x3_3x3(matrix *m1, matrix *m2, matrix *result){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == 3);
    assert(m2->numCols == 3);
    assert(result->numRows == 3);
    assert(result->numCols == 3);

    matrix *temp;

    if(m1 == result || m2 == result) {
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        zeroMatrix(temp);
    }

#pragma unroll
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                temp->data[(i * temp->numCols) + j] += m1->data[(i * m1->numCols) + k] * m2->data[(k * m2->numCols) + j];
            }
        }
    }

    if(m1 == result || m2 == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;

}




matrix *matMult_blas(matrix *m1, matrix *m2, matrix *result){
    assert(m1->numCols == m2->numRows);
    assert(result->numRows == m1->numRows);
    assert(result->numCols == m2->numCols);

    matrix *temp;
    if(m1 == result || m2 == result){
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        //zeroMatrix(temp);
    }

    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, m1->numRows, m2->numCols, m1->numCols, 1, m1->data, m1->numCols, m2->data, m2->numCols, 0, temp->data, temp->numCols);

    if(m1 == result || m2 == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;
}

//check for NAN in a matrix
int hasNan(matrix *m){

   for(int i = 0; i < m->numRows * m->numCols; i++) {
       if (isnan(m->data[i]))
           return 1;
   }
    return 0;

}


matrix *matMult_chain(matrix *m1, matrix *m2){

    assert(m1->numCols == m2->numRows);
    matrix *result = matrix_new(m1->numRows, m2->numCols);


    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            for(int k = 0; k < m1->numCols; k++){

                result->data[(i * result->numCols) + j] += m1->data[(i * m1->numCols) + j] * m2->data[(i * m2->numCols) + j];
                //assert(result->data[i][j] != NAN);
            }
        }
    }
    matrix_free(m1);
    matrix_free(m2);

    return result;
}

//matrix *matMult_alloc(matrix m1, matrix m2){
//
//    assert(m1.numCols == m2.numRows);
//    matrix *result = matrix_new(m1.numRows, m2.numCols);
//    for(int i = 0; i < m1.numRows; i++){
//        for(int j = 0; j < m2.numCols; j++){
//            for(int k = 0; k < m1.numCols; k++){
//
//                result->data[(i * result->numCols) + j] += m1.data[i][k] * m2.data[k][j];
//                //assert(result->data[i][j] != NAN);
//            }
//        }
//    }
//
//    return result;
//}

//matrix *matrix_outerProduct(matrix *m1, matrix *m2){
//    assert(m1->numCols == m2->numRows);
//    assert(m1->numRows == m2->numCols);
//    matrix *result = matrix_new(m1->numRows, m2->numCols);
//    for(int i = 0; i < m1->numRows; i++){
//        for(int j = 0; j < m2->numCols; j++){
//            result->data[i][j] = m1->data[i][j] * m2->data[j][i];
//        }
//    }
//    return result;
//
//}

matrix *matMult_elem(matrix *m1, matrix *m2){
    assert(m1->numCols == m2->numCols);
    assert(m1->numRows == m2->numRows);
    matrix *result = matrix_new(m1->numRows, m1->numCols);
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result->data[(i * result->numCols) + j] = m1->data[(i * m1->numCols) + j] * m2->data[(i * m2->numCols) + j];
        }
    }

    return result;
}

matrix *scalarMatDiv(matrix *m, double scalar, matrix *result){
    //matrix *result = matrix_new(m->numRows, m->numCols);
    if(scalar == 0){
        printf("DIVIDE BY ZERO");
        assert(0);
        return result;
    }
    matrix *temp;
    if(m == result) {
        temp = matrix_new(result->numRows, result->numCols);
    }else{
        temp = result;
        //zeroMatrix(temp);
    }
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            temp->data[(i * temp->numCols) + j] = m->data[(i * m->numCols) + j] / scalar;

        }
    }

    if(m == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;

}


double norm(matrix *m){
    double result = 0;

    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result += m->data[(i * m->numCols) + j] * m->data[(i * m->numCols) + j];
        }
    }

    if(result < 0){
        printf("NEGATIVE SQUARE ROOT");
        assert(0);
    }
    result = sqrt(result);
    return result;
}

void printMatrix(matrix *m){
   for(int i = 0; i < m->numRows; i++){
        printf("|");
        for(int j = 0; j < m->numCols; j++){
            printf("%.15f ", m->data[(i * m->numCols) + j]);
        }
        printf("|\n");
    }
}


matrix *matrixFromFile(char *filename, matrix *result){
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Could not open file");
        return NULL;
    }


    char buffer[result->numCols * 24]; //double is 15-17 sig-figs so about 24 chars max per cell
    int row = 0, col = 0;
    while (fgets(buffer, (int)sizeof(buffer), file)) {
        col = 0;
        char *value = strtok(buffer, ",");
        while (value && col < result->numCols) {
            result->data[row * result->numCols + col] = atof(value);
            value = strtok(NULL, ",");
            ++col;
        }
        ++row;
    }

    fclose(file);
    return result;
}
void matrixToFile(matrix *m, char *filename){
    FILE *f = fopen(filename, "a");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    for (int i = 0; i < m->numRows; i++){
        for (int j = 0; j < m->numCols; j++){
            if(j == m->numCols - 1)
                fprintf(f, "%.15f", m->data[(i * m->numCols) + j]);
            else
                fprintf(f, "%.15f, ", m->data[(i * m->numCols) + j]);

        }
        fprintf(f, "\n");
    }
    fclose(f);
}

char* matrixToJson(matrix *m, char *version){

    char *result = malloc(10000);
    if(strcmp(version, "matlab") != 0){
        sprintf(result, "{\n\"rows\": %d,\n\"cols\": %d,\n\"data\": [", m->numRows, m->numCols);
    }else{
        sprintf(result, "%s [", result);
    }
    for(int i = 0; i < m->numRows; i++){
        if(m->numRows != 1 && m->numCols != 1) {
            sprintf(result, "%s[", result);
        }
        for(int j = 0; j < m->numCols; j++) {
            if(m->numRows == 1 || m->numCols == 1) {
                if(j == m->numCols - 1 && i == m->numRows - 1) {

                    if(m->data[(i * m->numCols) + j] >99999){
                        sprintf(result, "%s%.18e", result, m->data[(i * m->numCols) + j]);
                    }else {
                        sprintf(result, "%s%.18f", result, m->data[(i * m->numCols) + j]);
                    }
                }else{
                    if(m->data[(i * m->numCols) + j] >99999){
                        sprintf(result, "%s%.18e,", result, m->data[(i * m->numCols) + j]);
                    }else {
                        sprintf(result, "%s%.18f,", result, m->data[(i * m->numCols) + j]);
                    }
                }
            } else {
                if(j == m->numCols - 1) {
                    if(m->data[(i * m->numCols) + j] >99999){
                        sprintf(result, "%s%.18e", result, m->data[(i * m->numCols) + j]);
                    }else {
                        sprintf(result, "%s%.18f", result, m->data[(i * m->numCols) + j]);
                    }
                }else {
                    if(m->data[(i * m->numCols) + j] >99999){
                        sprintf(result, "%s%.18e,", result, m->data[(i * m->numCols) + j]);
                    }else {
                        sprintf(result, "%s%.18f,", result, m->data[(i * m->numCols) + j]);
                    }
                }
            }
            sprintf(result, "%s", result);
        }
        if(m->numRows != 1 && m->numCols != 1) {
            if (i == m->numRows - 1)
                sprintf(result, "%s]", result);
            else
                sprintf(result, "%s],\n", result);
        }
        //sprintf(result, "%s],\n", result);
    }

    if(strcmp(version, "matlab") != 0){
        sprintf(result, "%s]\n}", result);
    }else{
        sprintf(result, "%s ]", result);
    }

    sprintf(result, "%s\n", result);

    return result;
}

matrix *matrixPow(matrix *m, int power, matrix *result){
    assert(m->square == 1);
    matrix *temp;
    if(m == result){
        temp = matrix_new(m->numRows, m->numCols);
    }else{
        temp = result;
    }
    copyMatrix(m, temp);

    // If power is 1, result should be the same as the original matrix
    if(power == 1){
        copyMatrix(m, temp);

    }else {
        matrix *tempResult = matrix_new(m->numRows, m->numCols);
        for (int i = 1; i < power; i++) {
            zeroMatrix(tempResult);
            matMult(m, temp, tempResult); // Multiply original matrix with temp
            copyMatrix(tempResult, temp); // Update temp with the result

        }
        matrix_free(tempResult); // Free the intermediate result
    }
    // Copy the final result into the output matrix
    if(m == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }

    return result;
}
matrix *matrix_sub(matrix *m1, matrix *m2, matrix *result){

    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);

    assert(result->numRows == m1->numRows);
    assert(result->numCols == m1->numCols);

    matrix *temp;
    if(m1 == result || m2 == result){
        temp = matrix_new(result->numRows, result->numCols);
        //copyMatrix(result, temp);
    }else{
        temp = result;
        //zeroMatrix(temp);
    }


    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            temp->data[(i * temp->numCols) + j] = m1->data[(i * temp->numCols) + j] - m2->data[(i * temp->numCols) + j];
        }
    }

    if(m1 == result || m2 == result){
        copyMatrix(temp, result);
        matrix_free(temp);
    }
    return result;
}


//matrix *matrix_sub_broadcast(matrix *m1, matrix *vect){
//    assert(m1->numRows == vect->numRows);
//    matrix *result = matrix_new(m1->numRows, m1->numCols);
//
//    for(int i = 0; i < m1->numRows; i++){
//        for(int j = 0; j < m1->numCols; j++){
//            result->data[i][j] = m1->data[i][j] - vect->data[i][0];
//        }
//    }
//    return result;
//}

double Det(matrix *m) {
    gsl_matrix *gsl_m = gsl_matrix_alloc(m->numRows, m->numCols);
    matrix_to_gsl(m, gsl_m);
    gsl_permutation *p = gsl_permutation_alloc(m->numRows);
    //gsl_matrix_view m4 = gsl_matrix_vi

    int signum;
    gsl_linalg_LU_decomp(gsl_m, p, &signum);
    //gsl_linalg_LU_decomp (a, p, &s);
    double det = gsl_linalg_LU_det (gsl_m, signum);
    gsl_matrix_free(gsl_m);
    gsl_permutation_free(p);

    if(isnan(det)){
       det = 0;
    }

    return det;
}

double gslDet(gsl_matrix *m) {
    gsl_permutation *p = gsl_permutation_alloc(m->size1);
    //gsl_matrix_view m4 = gsl_matrix_vi

    int signum;
    gsl_linalg_LU_decomp(m, p, &signum);
    //gsl_linalg_LU_decomp (a, p, &s);
    double det = gsl_linalg_LU_det (m, signum);
    gsl_permutation_free(p);

    if(isnan(det)){
        det = 0;
    }

    return det;
}

double matrixRatio(matrix *m1, matrix *m2){
    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    double result = 0;
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result += m1->data[(i * m1->numCols) + j] / m2->data[(i * m2->numCols) + j];
        }
    }
    return result;
}

matrix *elemDiv(matrix *m1, double scalar, matrix *result){
    //matrix *result = matrix_new(m1->numRows, m1->numCols);
    if(scalar == 0){
        printf("DIVIDE BY ZERO");
        assert(0);
        return result;
    }


    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result->data[(i * result->numCols) + j] = m1->data[(i * result->numCols) + j] / scalar;
        }
    }
    return result;
}



void printGSLMatrix(gsl_matrix *m){
    for(int i = 0; i < m->size1; i++){
        printf("|");
        for(int j = 0; j < m->size2; j++){
            printf("%.12f ", gsl_matrix_get(m, i, j));
        }
        printf("|\n");
    }
}



