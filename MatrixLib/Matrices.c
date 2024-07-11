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
matrix *matrix_new(uint8_t num_rows, uint8_t num_cols) {
    assert(num_rows != 0);
    assert(num_cols != 0);

    matrix *m = malloc(sizeof(matrix));
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
    //gsl_matrix_view m4 = gsl_matrix_vi

    int signum;
    gsl_linalg_LU_decomp(gsl_m, p, &signum);
    //gsl_linalg_LU_decomp (a, p, &s);
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



matrix *eye(matrix *result){

    assert(result->numRows == result->numCols);

    memset(result->data, 0, result->numRows * result->numCols * sizeof(double));
    for(int i = 0; i < result->numRows; i++){
        result->data[(i * result->numCols) + i] = 1;
    }
    return result;
}

//matrix *matrix_diag(matrix *A){//todo this is wrong
//    assert(A->numRows == A->numCols);
//    matrix *m = matrix_new(A->numRows, 1);
//    for(int i = 0; i < A->numRows; i++){
//        m->data[i][0] = A->data[i][i];
//    }
//    return m;
//
//}

matrix *matrix_rand(uint8_t num_rows, uint8_t num_cols){
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
matrix *zeros(uint8_t num_rows, uint8_t num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            m->data[(i * m->numCols) + j] = 0;
        }
    }
    return m;
}

//todo I am sure there is some bitwise wizardry that can be done here
matrix *zeroMatrix(matrix *m){
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            m->data[(i * m->numCols) + j] = 0;
        }
    }
    return m;
}

matrix *ones(uint8_t num_rows, uint8_t num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            m->data[(i * m->numCols) + j] = 1;
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
            m->data[(i * m->numCols) + j] = section->data[((i-startRow) * section->numCols) + (j-startCol)];
        }
    }


}


void copyMatrix(matrix *m, matrix *result){
    assert(m->numRows == result->numRows);
    assert(m->numCols == result->numCols);
    //zeroMatrix(result);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){

            result->data[(i * m->numCols) + j] = m->data[(i * m->numCols) + j];
        }
    }
}
void getSetSection(matrix *get, matrix *set, uint8_t getStartRow, uint8_t getEndRow, uint8_t getStartCol, uint8_t getEndCol, uint8_t setStartRow, uint8_t setEndRow, uint8_t setStartCol, uint8_t setEndCol) {
    // Check if input parameters are within bounds
    assert(getStartRow <= getEndRow && getStartCol <= getEndCol);
    assert(getEndRow < get->numRows && getEndCol < get->numCols);
    assert(setStartRow <= setEndRow && setStartCol <= setEndCol);
    assert(setEndRow < set->numRows && setEndCol < set->numCols);


    for (uint8_t i = 0; i <= getEndRow - getStartRow; i++) {
        for (uint8_t j = 0; j <= getEndCol - getStartCol; j++) {
            // Calculate indices for the source and destination matrices
            uint8_t src_i = getStartRow + i;
            uint8_t src_j = getStartCol + j;
            uint8_t dest_i = setStartRow + i;
            uint8_t dest_j = setStartCol + j;

            // Copy data from the source to the destination matrix
            set->data[(dest_i * set->numCols) + dest_j] = get->data[(src_i * get->numCols) + src_j];
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
            result->data[((i-startRow) * result->numCols) + (j- startCol)] = m->data[(i * m->numCols) + j];

        }
    }
    result->numRows = (endRow - startRow)+1;
    result->numCols = (endCol - startCol)+1;

    return result;
}

matrix *matrix_sin(matrix *m){//todo update to use result matrix
    matrix *result = matrix_new(m->numRows, m->numCols);
    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[(i * result->numCols) + j] =(double) sinl(m->data[(i * m->numCols) + j]);
        }
    }
    return result;

}
//todo this can be improved with different algorithms, matlab uses strassen's algorithm
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
    // Initialize the temporary/result matrix to zero
    //zeroMatrix(temp);

    // Perform the matrix multiplication
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
            }
        }
    }
    matrix_free(m1);
    matrix_free(m2);

    return result;
}


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

//todo is this right?
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
    while (fgets(buffer, sizeof(buffer), file)) {
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
////this only works for column vectors right now
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



