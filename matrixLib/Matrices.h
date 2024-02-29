#include <stdlib.h>

#ifndef _MATRICES_H_
#define _MARTRICES_H_
 


//Define a matrix of doubles
//max number of rows or columns is 255
typedef struct matrix_double_s {
    //there is probably some memory trickery that can be done here to make this more efficient
    uint8_t numRows;
    uint8_t numCols;
    double **data;
    uint8_t square;



} matrix;


// Constructor-like 
// Allocates memory for a new matrix
// All elements in the matrix are 0.0
matrix *matrix_new(uint8_t num_rows, uint8_t num_cols);


int *matrix_shape(matrix *m);

// Dot product
matrix *dot(matrix *m1, matrix *m2);

// Cross product
matrix *cross(matrix *m1, matrix *m2);

// Transpose
matrix *matrix_transpose(matrix *m);

// Addition
matrix *matrix_add(matrix *m1, matrix *m2);

// Subtraction
matrix *matrix_sub(matrix *m1, matrix *m2);

// Scalar multiplication
matrix *matrix_scalar_mul(matrix *m, float scalar);

// Identity matrix of size nxn
matrix *eye(uint8_t n);

matrix *diag(float *d, uint8_t n);

//multiply two matrices to create a new matrix
matrix *matMult(matrix *m1, matrix *m2);


matrix *matDiv(matrix *m1, matrix *m2);

matrix *zeros(uint8_t num_rows, uint8_t num_cols);

matrix *ones(uint8_t num_rows, uint8_t num_cols);


void printMatrix(matrix *m);

double norm(matrix *m);

//set a section of a matrix to another matrix. m is the matrix to be added to and section is the matrix to be added
void setSection(matrix *m, uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol, matrix *section);

matrix *getSection(matrix *m, uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol);



matrix *eigenvalues(matrix *m, int iterations);
// Destructor-like
void matrix_free(matrix *m);
#endif