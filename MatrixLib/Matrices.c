#include <assert.h>
#include <math.h>
#include <printf.h>
#include "Matrices.h"
#include <string.h>
#include <gsl/gsl_linalg.h>



gsl_matrix *matrix_to_gsl(matrix *matrix, gsl_matrix *out){
    //gsl_matrix *m = gsl_matrix_alloc(matrix->numRows, matrix->numCols);
    for(int i = 0; i < matrix->numRows; i++){
        for(int j = 0; j < matrix->numCols; j++){
            gsl_matrix_set(out, i, j, matrix->data[i][j]);
        }
    }
    return out;
}

matrix *expm(matrix *A, matrix *result){
    assert(A->square == 1);

    gsl_matrix *gsl_A = gsl_matrix_alloc(A->numRows, A->numCols);
    gsl_matrix *gsl_result = gsl_matrix_alloc(A->numRows, A->numCols);
    gsl_linalg_exponential_ss(gsl_A, gsl_result, GSL_PREC_DOUBLE);

    gsl_to_matrix(gsl_result, result);
    gsl_matrix_free(gsl_A);
    gsl_matrix_free(gsl_result);
    return result;
}

matrix *gsl_to_matrix(gsl_matrix *gsl_matrix, matrix *result){
    //matrix *m = matrix_new(gsl_matrix->size1, gsl_matrix->size2);
    for(int i = 0; i < result->numRows; i++){
        for(int j = 0; j < result->numCols; j++){
            result->data[i][j] = gsl_matrix_get(gsl_matrix, i, j);
        }
    }
    return result;
}
//make sure this is freed!!!
matrix *matrix_new(uint8_t num_rows, uint8_t num_cols) {
    assert(num_rows != 0);
    assert(num_cols != 0);

    // Allocate memory for the matrix struct
    matrix *m = malloc(sizeof(matrix));
    if (m == NULL) {
        fprintf(stderr, "Failed to allocate memory for matrix structure.\n");
        return NULL;
    }

    m->numCols = num_cols;
    m->numRows = num_rows;
    m->square = (num_cols == num_rows) ? 1 : 0;

    // Allocate memory for the row pointers
    m->data = malloc(m->numRows * sizeof(*(m->data)));
    if (m->data == NULL) {
        fprintf(stderr, "Failed to allocate memory for matrix rows.\n");
        free(m);
        return NULL;
    }

    // Allocate memory for each row and initialize to zero
    for (int i = 0; i < num_rows; i++) {
        m->data[i] = calloc(num_cols, sizeof(**m->data));
        if (m->data[i] == NULL) {
            fprintf(stderr, "Failed to allocate memory for matrix row %d.\n", i);
            // Free previously allocated rows
            for (int j = 0; j < i; j++) {
                free(m->data[j]);
            }
            free(m->data);
            free(m);
            return NULL;
        }
    }

    return m;
}



void matrix_free(matrix *m) {
    assert(m != NULL);
    if (m->data != NULL) {
        for (int i = 0; i < m->numRows; i++) {
            if (m->data[i] != NULL) {
                free(m->data[i]);
            }
        }
        free(m->data);
    }
    free(m);
}


matrix *matrix_add(matrix *m1, matrix *m2, matrix *result){
    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    //matrix *result = matrix_new(m1->numRows, m1->numCols);
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result->data[i][j] = m1->data[i][j] + m2->data[i][j];
        }
    }
    return result;

}

matrix *matrix_add3(matrix *m1,matrix *m2, matrix *m3, matrix *result){

    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    assert(m1->numRows == m3->numRows);
    assert(m1->numCols == m3->numCols);

    matrix_add(m1, m2, result);
    matrix_add(result, m3, result);

    return result;
}

matrix *matrix_scalar_mul(matrix *m, double scalar, matrix *result){
    //matrix *result = matrix_new(m->numRows, m->numCols);

    assert(result->numRows == m->numRows);
    assert(result->numCols == m->numCols);

    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[i][j] = m->data[i][j] * scalar;
        }
    }
    return result;
}

matrix *matrix_scalar_mul_chain(matrix *m, double scalar){
    matrix *result = matrix_new(m->numRows, m->numCols);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[i][j] = m->data[i][j] * scalar;
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

    return shape;
}



//this might need a transpose on the inverse
matrix *matrix_solve(matrix *A, matrix *b, matrix *result){
    assert(A->numRows == b->numRows);
    assert(A->numCols == A->numRows);
    //matrix *result = matrix_new(A->numRows, 1);

    matrix *A_inv = matrix_new(A->numRows, A->numCols);
    matrix_inverse(A, A_inv);
    matMult(A_inv, b, result);

    matrix_free(A_inv);

    return result;
}

matrix *matrix_inverse(matrix *m, matrix *result){
    assert(m->square == 1);
    gsl_matrix *gsl_m = gsl_matrix_alloc(m->numRows, m->numCols);
    matrix_to_gsl(m, gsl_m);
    gsl_permutation *p = gsl_permutation_alloc(m->numRows);
    //gsl_matrix_view m4 = gsl_matrix_vi

    int signum;
    gsl_linalg_LU_decomp(gsl_m, p, &signum);
    //gsl_linalg_LU_decomp (a, p, &s);
    gsl_linalg_LU_invx(gsl_m, p);
    gsl_to_matrix(gsl_m, result);
    gsl_matrix_free(gsl_m);
    gsl_permutation_free(p);

//    for(int i = 0; i < result->numRows; i++){
//        for(int j = 0; j < result->numCols; j++){
//            assert(!isnan(result->data[i][j]));
//        }
//    }


//    matrix *result = matrix_new(m->numRows, m->numCols);
//    double det = Det(m);
//    assert(det != 0);
//    for(int i = 0; i < m->numRows; i++){
//        for(int j = 0; j < m->numCols; j++){
//            matrix *sub = matrix_new(m->numRows - 1, m->numCols - 1);
//            for(int k = 0; k < m->numRows; k++){
//                for(int l = 0; l < m->numCols; l++){
//                    if(k < i){
//                        if(l < j){
//                            sub->data[k][l] = m->data[k][l];
//                        }else if(l > j){
//                            sub->data[k][l-1] = m->data[k][l];
//                        }
//                    }else if(k > i){
//                        if(l < j){
//                            sub->data[k-1][l] = m->data[k][l];
//                        }else if(l > j){
//                            sub->data[k-1][l-1] = m->data[k][l];
//                        }
//                    }
//                }
//            }
//            result->data[i][j] = pow(-1, i+j) * Det(sub) / det;
//            matrix_free(sub);
//        }
//    }


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
            result += m1->data[i][j] * m2->data[j][i];
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
            result->data[i][j] = m1->data[i][j] * m2->data[j][i];
        }
    }

    return result;
}

matrix *matrix_transpose(matrix *m, matrix *result){
    //matrix *result = matrix_new(m->numCols, m->numRows);

    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[j][i] = m->data[i][j];
        }
    }

    return result;
}



matrix *eye(matrix *result){

    assert(result->numRows == result->numCols);
    //matrix *m = matrix_new(n,n);

    for(int i = 0; i < result->numRows; i++){
        memset(result->data[i], 0, result->numRows * sizeof(int));  // Set the whole row to zero
        result->data[i][i] = 1;
    }
    return result;
}

matrix *matrix_diag(matrix *A){//todo this is wrong
    assert(A->numRows == A->numCols);
    matrix *m = matrix_new(A->numRows, 1);
    for(int i = 0; i < A->numRows; i++){
        m->data[i][0] = A->data[i][i];
    }
    return m;

}

matrix *matrix_rand(uint8_t num_rows, uint8_t num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            m->data[i][j] = (double)rand() / RAND_MAX;
        }
    }
    return m;
}

double matrix_sumSelf(matrix *m){
    double result = 0;
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result += m->data[i][j];
        }
    }
    return result;
}
matrix *zeros(uint8_t num_rows, uint8_t num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            m->data[i][j] = 0;
        }
    }
    return m;
}

//todo I am sure there is some bitwise wizardry that can be done here
matrix *zeroMatrix(matrix *m){
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            m->data[i][j] = 0;
        }
    }
    return m;
}

matrix *ones(uint8_t num_rows, uint8_t num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            m->data[i][j] = 1;
        }
    }
    return m;
}


void setSection(matrix *m, uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol, matrix *section){
    assert(startRow <= endRow);
    assert(startCol <= endCol);
    assert(endRow <= m->numRows);
    assert(endCol <= m->numCols);
    //assert(section->numRows -1 == (endRow - startRow));
    //assert(section->numCols -1 == (endCol - startCol));

    for(int i = startRow; i <= endRow; i++){
        for(int j = startCol; j <= endCol; j++){
            m->data[i][j] = section->data[i - startRow][j - startCol];
        }
    }


}


void copyMatrix(matrix *m, matrix *result){
    assert(m->numRows == result->numRows);
    assert(m->numCols == result->numCols);

    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[i][j] = m->data[i][j];
        }
    }
}
void getSetSection(matrix *get, matrix *set, uint8_t getStartRow, uint8_t getEndRow, uint8_t getStartCol, uint8_t getEndCol, uint8_t setStartRow, uint8_t setEndRow, uint8_t setStartCol, uint8_t setEndCol){
    assert(getStartRow <= getEndRow);
    assert(getStartCol <= getEndCol);
    assert(getEndRow <= get->numRows);
    assert(getEndCol <= get->numCols);

    assert(setStartRow <= setEndRow);
    assert(setStartCol <= setEndCol);
    //todo fix these asserts
    assert(setEndRow <= set->numRows+1);
    assert(setEndCol <= set->numCols+1);


    int ii = setStartRow;
    int jj = setStartCol;

    for(int i = getStartRow; i <= getEndRow; i++){
        ii++;
        for(int j = getStartCol; j <= getEndCol; j++){
            jj++;
            set->data[i][j] = get->data[ii][jj];
            //memcpy(set->data[i - setStartRow], get->data[i], sizeof(double) * (getEndCol - getStartCol));
        }
    }

}

matrix *getSection(matrix *m, uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol, matrix *result){

    assert(startRow <= endRow);
    assert(startCol <= endCol);
    assert(endRow <= m->numRows);
    assert(endCol <= m->numCols);

    assert(result->numRows == (endRow - startRow)+1);
    assert(result->numCols == (endCol - startCol)+1);
    //matrix *section = matrix_new((endRow - startRow)+1, (endCol - startCol)+1);//todo is this right? should it add one?

    for(int i = startRow; i <= endRow; i++){
        for(int j = startCol; j <= endCol; j++){
            result->data[i - startRow][j - startCol] = m->data[i][j];
            assert(result->data[i - startRow][j - startCol] != NAN);
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
            result->data[i][j] =(double) sinl(m->data[i][j]);
        }
    }
    return result;

}
//todo this can be improved with different algorithms, matlab uses strassen's algorithm
matrix *matMult(matrix *m1, matrix *m2, matrix *result){

    assert(m1->numCols == m2->numRows);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            for(int k = 0; k < m1->numCols; k++){

                result->data[i][j] += m1->data[i][k] * m2->data[k][j];
                //assert(result->data[i][j] != NAN);
            }
        }
    }

    return result;
}

matrix *matMult_chain(matrix *m1, matrix *m2){

    assert(m1->numCols == m2->numRows);
    matrix *result = matrix_new(m1->numRows, m2->numCols);
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            for(int k = 0; k < m1->numCols; k++){

                result->data[i][j] += m1->data[i][k] * m2->data[k][j];
                //assert(result->data[i][j] != NAN);
            }
        }
    }
    matrix_free(m1);
    matrix_free(m2);

    return result;
}

matrix *matMult_alloc(matrix m1, matrix m2){

    assert(m1.numCols == m2.numRows);
    matrix *result = matrix_new(m1.numRows, m2.numCols);
    for(int i = 0; i < m1.numRows; i++){
        for(int j = 0; j < m2.numCols; j++){
            for(int k = 0; k < m1.numCols; k++){

                result->data[i][j] += m1.data[i][k] * m2.data[k][j];
                //assert(result->data[i][j] != NAN);
            }
        }
    }

    return result;
}

matrix *matrix_outerProduct(matrix *m1, matrix *m2){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == m2->numCols);
    matrix *result = matrix_new(m1->numRows, m2->numCols);
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            result->data[i][j] = m1->data[i][j] * m2->data[j][i];
        }
    }
    return result;

}

matrix *matMult_elem(matrix *m1, matrix *m2){
    assert(m1->numCols == m2->numCols);
    assert(m1->numRows == m2->numRows);
    matrix *result = matrix_new(m1->numRows, m1->numCols);
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result->data[i][j] = m1->data[i][j] * m2->data[i][j];
        }
    }

    return result;
}

matrix *scalarMatDiv(matrix *m, double scalar, matrix *result){
    //matrix *result = matrix_new(m->numRows, m->numCols);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[i][j] = m->data[i][j] / scalar;
        }
    }
    return result;

}

//todo is this right?
double norm(matrix *m){
    double result = 0;

    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result += m->data[i][j] * m->data[i][j];
        }
    }

    result = sqrt(result);
    return result;
}

void printMatrix(matrix *m){
   for(int i = 0; i < m->numRows; i++){
        printf("|");
        for(int j = 0; j < m->numCols; j++){
            printf("%.15f ", m->data[i][j]);
        }
        printf("|\n");
    }
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
            fprintf(f, "%.12f, ", m->data[i][j]);
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

                    if(m->data[i][j] >99999){
                        sprintf(result, "%s%.18e", result, m->data[i][j]);
                    }else {
                        sprintf(result, "%s%.18f", result, m->data[i][j]);
                    }
                }else{
                    if(m->data[i][j] >99999){
                        sprintf(result, "%s%.18e,", result, m->data[i][j]);
                    }else {
                        sprintf(result, "%s%.18f,", result, m->data[i][j]);
                    }
                }
            } else {
                if(j == m->numCols - 1) {
                    if(m->data[i][j] >99999){
                        sprintf(result, "%s%.18e", result, m->data[i][j]);
                    }else {
                        sprintf(result, "%s%.18f", result, m->data[i][j]);
                    }
                }else {
                    if(m->data[i][j] >99999){
                        sprintf(result, "%s%.18e,", result, m->data[i][j]);
                    }else {
                        sprintf(result, "%s%.18f,", result, m->data[i][j]);
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

    //result->data = m->data;
    for(int i = 0; i< m->numRows; i++){
        memcpy(result->data[i], m->data[i], m->numCols * sizeof(double));
    }
    //memcpy(result->data, m->data, m->numRows * m->numCols * sizeof(double));
    result->square = 1;
//    for(int i = 0; i < m->numRows; i++){
//        for(int j = 0; j < m->numCols; j++){
//            result->data[i][j] = m->data[i][j];
//        }
//    }
    for(int i = 0; i < power-1; i++){
        matMult(result, m, result);
    }
    return result;
}
matrix *matrix_sub(matrix *m1, matrix *m2, matrix *result){

    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);

    assert(result->numRows == m1->numRows);
    assert(result->numCols == m1->numCols);

    //matrix *result = matrix_new(m1->numRows, m1->numCols);
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result->data[i][j] = m1->data[i][j] - m2->data[i][j];
        }
    }
    return result;
}
//this only works for column vectors right now
matrix *matrix_sub_broadcast(matrix *m1, matrix *vect){
    assert(m1->numRows == vect->numRows);
    matrix *result = matrix_new(m1->numRows, m1->numCols);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result->data[i][j] = m1->data[i][j] - vect->data[i][0];
        }
    }
    return result;
}
double slowDet(matrix *m){
    assert(m->square == 1);
    matrix *sub = matrix_new(m->numRows - 1, m->numCols - 1);
    if(m->numRows == 1){
        matrix_free(sub);
        return m->data[0][0];
    }else if(m->numRows == 2){
        matrix_free(sub);
        return m->data[0][0] * m->data[1][1] - m->data[0][1] * m->data[1][0];
    }else{
        double result = 0;
        for(int i = 0; i < m->numRows; i++){
            sub = matrix_new(m->numRows - 1, m->numCols - 1);
            for(int j = 1; j < m->numRows; j++){
                for(int k = 0; k < m->numCols; k++){
                    if(k < i){
                        sub->data[j-1][k] = m->data[j][k];
                    }else if(k > i){
                        sub->data[j-1][k-1] = m->data[j][k];
                    }
                }
            }
            result += pow(-1, i) * m->data[0][i] * slowDet(sub);

        }

        matrix_free(sub);
        return result;
    }


}

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



double matrixRatio(matrix *m1, matrix *m2){
    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    double result = 0;
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result += m1->data[i][j] / m2->data[i][j];
        }
    }
    return result;
}

matrix *elemDiv(matrix *m1, double scalar, matrix *result){
    //matrix *result = matrix_new(m1->numRows, m1->numCols);
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result->data[i][j] = m1->data[i][j] / scalar;
        }
    }
    return result;
}


//This finds the largest eigenvector of a matrix (todo this uses power iteration, does it always land on the largest?)
matrix *eigenvector(matrix *A, int iterations) {
    int *shape = matrix_shape(A);
    int n;
    int m;
    if (shape[0] > shape[1]) {
        n = shape[0];
        m = shape[1];
    } else {
        n = shape[1];
        m = shape[0];
    }

    double vals[n];
    matrix *b_k = matrix_new(n, m);//eigen vector
    matrix *b_k1 = matrix_new(n, m);


//    b_k->data[0][0] = (double) rand() / RAND_MAX;
//    b_k->data[1][0] = (double) rand() / RAND_MAX;
//    b_k->data[2][0] = (double) rand() / RAND_MAX;

    for(int i = 0; i < n; i++){
        for(int j = 0; j < m; j++) {
            b_k->data[i][j] = (double) rand() / RAND_MAX;
        }
    }
    double b_k1_norm;

    while (iterations >= 0) {
        matMult(A, b_k, b_k1);

        //printMatrix(b_k1);

        b_k1_norm = norm(b_k1);


        scalarMatDiv(b_k1, b_k1_norm, b_k);

        iterations--;
    }

    free(b_k1);
    free(shape);
    return b_k;

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

double eigenvalue(matrix *A, int iterations){
    matrix *v = eigenvector(A, iterations);
    matrix *vT = matrix_new(v->numCols, v->numRows);
    matrix_transpose(v, vT);
    matrix *vTv = matrix_new(v->numCols, v->numCols);
    matMult(vT, v, vTv);

    matrix *temp = matrix_new(v->numCols, v->numCols);
    double result = matrixRatio(matMult(vT, matMult(A,v, temp), temp), vTv );//todo does this work??

    matrix_free(v);
    matrix_free(vT);
    matrix_free(vTv);
    matrix_free(temp);
    return result;
}


