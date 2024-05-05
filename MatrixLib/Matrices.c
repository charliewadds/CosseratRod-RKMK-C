#include <assert.h>
#include <math.h>
#include <printf.h>
#include "Matrices.h"





gsl_matrix *matrix_to_gsl(matrix *matrix){
    gsl_matrix *m = gsl_matrix_alloc(matrix->numRows, matrix->numCols);
    for(int i = 0; i < matrix->numRows; i++){
        for(int j = 0; j < matrix->numCols; j++){
            gsl_matrix_set(m, i, j, matrix->data[i][j]);
        }
    }
    return m;
}

matrix *gsl_to_matrix(gsl_matrix *gsl_matrix){
    matrix *m = matrix_new(gsl_matrix->size1, gsl_matrix->size2);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            m->data[i][j] = gsl_matrix_get(gsl_matrix, i, j);
        }
    }
    return m;
}

matrix *matrix_new(uint8_t num_rows, uint8_t num_cols){


    assert(num_rows != 0);
    assert(num_cols != 0);

    matrix *m = malloc(sizeof(matrix));

    m->numCols = num_cols;
    m->numRows = num_rows;
    if(num_cols == num_rows){
        m->square = 1;
    }else{
        m->square = 0;
    }

    //allocate a pointer for each row
    
    m->data = malloc(m->numRows * sizeof(*(m->data)));
    //NP_CHECK(m->data);

//    for(int i = num_rows; i>=0; i--){
//
//        m->data[i]= calloc(num_cols, sizeof(**m->data));;
//        //NP_CHECK(m->data[i]);
//    }//0x600000514150
    for (int i = num_rows - 1; i >= 0; i--) {
        m->data[i] = calloc(num_cols, sizeof(**m->data));
        // NP_CHECK(m->data[i]);
    }

    return m;
}

void matrix_free(matrix *m){

    int i = m->numRows-1;
    for(i; i>=0; i--){
        if(m->data[i] != NULL){
            free(m->data[i]);
            m->data[i] = NULL;
        }else{
            printf("tried to free null pointer\n");//todo could I just break here?
        }


    }

    if(m->data != NULL) {
        free(m->data);
        m->data = NULL;
    }else{
        printf("tried to free null pointer\n");
    }
    if(m != NULL){
        free(m);
        m = NULL;
    }else{
        printf("tried to free null pointer\n");
    }


}


matrix *matrix_add(matrix* restrict m1, matrix* restrict m2){
    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    matrix *result = matrix_new(m1->numRows, m1->numCols);
    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m1->numCols; j++){
            result->data[i][j] = m1->data[i][j] + m2->data[i][j];
        }
    }
    return result;
}

matrix *matrix_scalar_mul(matrix *m, double scalar){
    matrix *result = matrix_new(m->numRows, m->numCols);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[i][j] = m->data[i][j] * scalar;
        }
    }
    return result;
}

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
matrix *matrix_solve(matrix *A, matrix *b){
    assert(A->numRows == b->numRows);
    assert(A->numCols == A->numRows);
    matrix *result = matrix_new(A->numRows, 1);
    matrix *A_inv = matrix_inverse(A);
    result = dot(A_inv, b);
    return result;
}

matrix *matrix_inverse(matrix *m){
    assert(m->square == 1);
    matrix *result = matrix_new(m->numRows, m->numCols);
    double det = Det(m);
    assert(det != 0);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            matrix *sub = matrix_new(m->numRows - 1, m->numCols - 1);
            for(int k = 0; k < m->numRows; k++){
                for(int l = 0; l < m->numCols; l++){
                    if(k < i){
                        if(l < j){
                            sub->data[k][l] = m->data[k][l];
                        }else if(l > j){
                            sub->data[k][l-1] = m->data[k][l];
                        }
                    }else if(k > i){
                        if(l < j){
                            sub->data[k-1][l] = m->data[k][l];
                        }else if(l > j){
                            sub->data[k-1][l-1] = m->data[k][l];
                        }
                    }
                }
            }
            result->data[i][j] = pow(-1, i+j) * Det(sub) / det;
            matrix_free(sub);
        }
    }
    return result;
}
//todo does this need to be dynamically allocated?
matrix *dot(matrix *m1, matrix *m2){
    assert(m1->numCols == m2->numRows);

    matrix *result = matrix_new(m1->numRows, m2->numCols);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            for(int k = 0; k < m1->numCols; k++){
                result->data[i][j] += m1->data[i][k] * m2->data[k][j];
            }
        }
    }

    return result;
}

matrix *cross(matrix *m1, matrix *m2){
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

matrix *matrix_transpose(matrix *m){
    matrix *result = matrix_new(m->numCols, m->numRows);

    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[j][i] = m->data[i][j];
        }
    }

    return result;
}



matrix *eye(uint8_t n){
    matrix *m = matrix_new(n,n);
    for(int i = 0; i < n; i++){
        m->data[i][i] = 1;
    }
    return m;
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

matrix *getSection(matrix *m, uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol){

    assert(startRow <= endRow);
    assert(startCol <= endCol);
    assert(endRow <= m->numRows);
    assert(endCol <= m->numCols);

    matrix *section = matrix_new((endRow - startRow)+1, (endCol - startCol)+1);//todo is this right? should it add one?

    for(int i = startRow; i <= endRow; i++){
        for(int j = startCol; j <= endCol; j++){
            section->data[i - startRow][j - startCol] = m->data[i][j];
            assert(section->data[i - startRow][j - startCol] != NAN);
        }
    }
    section->numRows = (endRow - startRow)+1;
    section->numCols = (endCol - startCol)+1;

    return section;
}

matrix *matrix_sin(matrix *m){
    matrix *result = matrix_new(m->numRows, m->numCols);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[i][j] = sin(m->data[i][j]);
        }
    }
    return result;

}
//todo this can be improved with different algorithms, matlab uses strassen's algorithm
matrix *matMult(matrix *m1, matrix *m2){
    //printf("shape of m1 [%f, %f]\n", m1->numRows, m1->numCols);
    //printf("shape of m1 [%f, %f]\n", m1->numRows, m1->numCols);
    assert(m1->numCols == m2->numRows);
    //assert(m1->numRows == m2->numCols);todo I dont need this right?

    matrix *result = matrix_new(m1->numRows, m2->numCols);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            for(int k = 0; k < m1->numCols; k++){

                result->data[i][j] += m1->data[i][k] * m2->data[k][j];
                assert(result->data[i][j] != NAN);
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

matrix *scalarMatDiv(matrix *m, double scalar){
    matrix *result = matrix_new(m->numRows, m->numCols);
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
            printf("%.12f ", m->data[i][j]);
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

matrix *matrixPow(matrix *m, int power){
    assert(m->square == 1);
    matrix *result = matrix_new(m->numRows, m->numCols);
    result->data = m->data;
    result->square = 1;
//    for(int i = 0; i < m->numRows; i++){
//        for(int j = 0; j < m->numCols; j++){
//            result->data[i][j] = m->data[i][j];
//        }
//    }
    for(int i = 0; i < power-1; i++){
        result = matMult(result, m);
    }
    return result;
}
matrix *matrix_sub(matrix *m1, matrix *m2){

    assert(m1->numRows == m2->numRows);
    assert(m1->numCols == m2->numCols);
    matrix *result = matrix_new(m1->numRows, m1->numCols);
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
double Det(matrix *m){
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
            result += pow(-1, i) * m->data[0][i] * Det(sub);

        }

        matrix_free(sub);
        return result;
    }


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

matrix *elemDiv(matrix *m1, double scalar){
    matrix *result = matrix_new(m1->numRows, m1->numCols);
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
    if (shape[0] > shape[1]) {
        n = shape[0];
    } else {
        n = shape[1];
    }

    double vals[n];
    matrix *b_k = matrix_new(n, 1);//eigen vector
    matrix *b_k1;// = matrix_new(n, 1);


    b_k->data[0][0] = (double) rand() / RAND_MAX;
    b_k->data[1][0] = (double) rand() / RAND_MAX;
    b_k->data[2][0] = (double) rand() / RAND_MAX;
    double b_k1_norm;

    while (iterations >= 0) {
        b_k1 = dot(A, b_k);

        //printMatrix(b_k1);

        b_k1_norm = norm(b_k1);


        b_k = scalarMatDiv(b_k1, b_k1_norm);

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
    matrix *vT = matrix_transpose(v);
    matrix *vTv = dot(vT, v);

    double result = matrixRatio(dot(vT, dot(A,v)), vTv );
    matrix_free(v);
    matrix_free(vT);
    matrix_free(vTv);
    return result;
}


